/*
 * proc_task.c
 *
 *  Created on: Apr 3, 2022
 *      Author: SA4P Authors
 */

#include "proc_task.h"

uint16_t proc_get_access_type(byte_t* payload);
bool_t	 proc_set_req_state(uint16_t access_type, req_t *req_state, crypto_context_t* keys_struct, cntrs_t* cntrs_struct);
err_t	 proc_check_response(msg_t* msg_state, req_t* req_state, crypto_context_t* keys_struct);


bool_t task_process_msg(msg_t* msg_state, req_t* req_state, crypto_context_t* keys_struct, cntrs_t* cntrs_struct, send_msg_t* o_msg){
	flag_msg_rdy = FALSE;

	if(msg_state->payload_type == PAYLOAD_SIGNUP_RESP){
		// Received a response to a signup request, but we are already signed up ==> Re-pairing
		// NOTE: Once this call returns, the cryptographic state will have ALWAYS been overwritten! (errors in this call are caught in an infinite loop within the function)
		pairing_process_second_message(msg_state->p_payload);

		// Send dummy request to server for key confirmation (create it and tell send task by setting flag that we want to send it)
		proc_set_req_state(0x69, &req_state, &cc_struct, &cntrs_struct);
		msg_build(PAYLOAD_CHALLENGE, &req_state, o_msg);
		flag_send_message = TRUE;

	}

	if(msg_state->payload_type == PAYLOAD_REQUEST){
		// Get access type from payload
		#ifdef PROFILING_REQUEST
//			PROFILING_SET_PIN(gpio_profiling_pin_2);
			// Start profiling...
			PROFILING_SET_COUNTER_F(0);
		#endif

		#ifdef PROFILING_MACRO_AUTH_TIME
			// 0: Started processing request
			PROFILING_SET_COUNTER_F(0);
		#endif

		uint16_t access_type = proc_get_access_type(msg_state->p_payload);

		// Check whether we are currently already serving a request and if it has the same access type
		if(flag_currently_granting == TRUE){

			// We subsequently check whether the new request is for the same access type.
			// If not, we do NOT process the request because we are currently granting another access type
			// so we cannot do the "timer-reset" technique and we only allow one access type at a time.
			if(access_type != flag_type_currently_granting){
				// If we reach here, the request type was different
				return FALSE;
			}

			// If we reached here, we have a request of the same type as is currently being granted
			__NOP();
		}

		#ifdef PROFILING_REQUEST
			// State: Checked if we are already currently granting, and if so, if it has the correct access type...
			PROFILING_SET_COUNTER_F(1);
		#endif

		// Set gateway's request state accordingly (incl. challenge).
		// This can lead to then request state being overwritten, but ONLY if we are not currently granting!
		bool_t success = proc_set_req_state(access_type, req_state, keys_struct, cntrs_struct);

		#ifdef PROFILING_REQUEST
			// State: Finished setting new request state...
			PROFILING_SET_COUNTER_F(4);
		#endif

		// Check if setting was successful, i.e. if the access type is in the range
		if(!success){
			return FALSE;
		}

		// Generate message
		if(!msg_build(PAYLOAD_CHALLENGE, req_state, o_msg)){
			while(TRUE){
				__NOP();
			}
		}

		// Notify sender task to send the challenge
		flag_send_message = TRUE;

		#ifdef PROFILING_MACRO_AUTH_TIME
			// 1: Created challenge ==> Ready to send
			PROFILING_SET_COUNTER_F(1);
		#endif

		#ifdef PROFILING_REQUEST
			// State: Finished building response message...
			PROFILING_SET_COUNTER_F(5);
		#endif

		return TRUE;
	}

	if(msg_state->payload_type == PAYLOAD_RESPONSE){
		#ifdef PROFILING_RESPONSE
			// State: Received new response
			PROFILING_SET_COUNTER_F(0);
		#endif

	#ifdef PROFILING_MACRO_AUTH_TIME
		// 2: Received response
		PROFILING_SET_COUNTER_F(2);
	#endif

		if(0 ==  proc_check_response(msg_state, req_state, keys_struct)){															// proc_check_response returns an error code, 0 means no error, < 0 means error.

			#ifdef PROFILING_RESPONSE
				// State: Finished copying randomness from message into request state (to be used in the next request)
				//        ==> exited proc_check_response
				PROFILING_SET_COUNTER_F(5);
			#endif

			// We do not (yet) take the concrete error values into account, but instead just return whether there has been an error or not
			// Will start serving the request ==> Set its request state to false
			req_state->valid = FALSE;

			if(!msg_build(PAYLOAD_GRANTED, req_state, o_msg)){
				while(TRUE){
					__NOP();
				}
			}

			#ifdef PROFILING_RESPONSE
				// State: Finished building granted message
				PROFILING_SET_COUNTER_F(6);
			#endif

			// If we reached here, we can grant access.
			// We have two cases:
			// NOTE: To check in which case we are, we have to disable the the update event generation. Hence, we beforehand have to check if the shoudl REALLY reset/start the counter, as we could otherwise extend an access we should not!
			// (1) Currently already granting. By the checks performed in the request processing, we know that this response is to the same request type as is currently being granted
			//	   ==> Simply reset the timer to 0.
			// (2) Currently not granting. Then, we just set the timer, set flags "flag_type_currently_granting" and "flag_currently_granting" and then start the timer.

			bool_t equal_access_type = (req_state->access_type == flag_type_currently_granting);

			if(flag_currently_granting && !equal_access_type){
				// We should in principle NEVER reach here, this means that we granted a request despite currently processing another request. Because we do NOT do any pipelining, this is an illegal state.
				__NOP();
				return FALSE;
			}

			// If we reach here we can be 100% sure that we are currently not granting OR we are granting the same request type for which we just got a valid response ==> Can start (resp. reset count register of) timer!

			// Disable update event generation
			TIMER_SET_UDIS(TIM7);

			// Check if we are STILL granting (NOTE: an update event could have occurred between our preliminary check above and now)
			// Now, because we have the UDIS flag set, we can be 100% sure that the flag_currently_granting will not change because the interrupt won't be triggered due to the UDIS flag being set!

			if(flag_currently_granting){
				// If we reach here, we know that the timer is currently running and that (by the preliminary check above) we are currently serving the same access type.
				// Hence, we are in case (1). We have two subcases:
				// (1.1) We reset the UDIS flag before the timer runs out (i.e. before the access should be revoked)
				//       Normal case, the timer count register will just be reset
				// (1.2) A timer update event WOULD have been raised, but it was suppressed (already within the timer device ==> never reached CPU core) because the UDIS flag was set.
				//		 Hence, we actually give the requesting CPU slightly more time than allocated, namely the time between when the update event _would_ have been generated and us resetting the count register.
				//		 This time is likely in the order of single-digit microseconds!
				TIMER_RESET_CNT(TIM7);
			}else{
				// If we reach here, this is because the timer wasn't running in the first place
				flag_currently_granting = TRUE;
				flag_type_currently_granting = req_state->access_type;
				timer_start(TIM7, TIMEOUT_10);								// Time is given in milliseconds (e.g. 5000), but one tick of the timer is 100us ==> multiply with 10 to get from 5000 * 100 us to 5000 * 1000 us = 5000 ms
			}

			// Re-Enable update event generation
			TIMER_CLEAR_UDIS(TIM7);

			// Set flag to 1, which means we notify the main CPU about being able to access the sensor
			flag_send_message = TRUE;

			#ifdef PROFILING_MACRO_AUTH_TIME
				// 3: Validated response and created challenge ==> Ready to send
				PROFILING_SET_COUNTER_F(3);
			#endif

			#ifdef PROFILING_RESPONSE
				// State: Finished updating watchdog timer
				PROFILING_SET_COUNTER_F(7);
			#endif
			return TRUE;
		}
	}

	__NOP();

//	#ifdef PROFILING_RESPONSE
//		PROFILING_RESET_PIN(gpio_profiling_pin_3);
//	#endif

	return FALSE;
}

uint16_t proc_get_access_type(byte_t* payload){
	return ((uint16_t*)payload)[0];
}

bool_t proc_set_req_state(uint16_t access_type, req_t *req_state, crypto_context_t* keys_struct, cntrs_t* cntrs_struct){


	// (1) Check if access type is within range
	if ((access_type < 0 || access_type > CONTROL_ACTUATOR_1) && access_type != DUMMY_REQUEST){															// Received malformed access type ==> Ignore request
		__NOP();																											// NOP to break on for debugging purposes
		return FALSE;
	}

	// (2) Set access type
	req_state->access_type = access_type;

	// (3) Build challenge message. Recall, challenge message is: |  reb_cnt  |  req_cnt  |  access_type  |  mac_tag  |
	//     To ensure freshness, we also include the randomness from the last message by the server. This randomness is however not explicitly sent to the server.
	//     We use the randomness from the server's previous authentication response.
	//     This means that the mac_tag is actually computed over: | reb_cnt | req_cnt | access_type | randomness |.

	byte_t hmac_input_buf[REB_CNT_LEN + REQ_CNT_LEN + ACCESS_TYPE_LEN + RANDOM_LEN];
	memcpy(hmac_input_buf, &cntrs_struct->reb_cnt, REB_CNT_LEN);
	memcpy(hmac_input_buf + REB_CNT_LEN, &cntrs_struct->req_cnt, REQ_CNT_LEN);
	memcpy(hmac_input_buf + REB_CNT_LEN + REQ_CNT_LEN, &access_type, ACCESS_TYPE_LEN);
	memcpy(hmac_input_buf + REB_CNT_LEN + REQ_CNT_LEN + ACCESS_TYPE_LEN, req_state->randomness, RANDOM_LEN);


//	memcpy(req_state->challenge, &cntrs_struct->reb_cnt, REB_CNT_LEN);
//	memcpy(req_state->challenge + REB_CNT_LEN, &cntrs_struct->req_cnt, REQ_CNT_LEN);
//	memcpy(req_state->challenge + REB_CNT_LEN + REQ_CNT_LEN, &access_type, ACCESS_TYPE_LEN);

	// This is the MAC-Tag offset in the CHALLENGE message, see comment for step (3).
	uint32_t mac_tag_offset = REB_CNT_LEN + REQ_CNT_LEN + ACCESS_TYPE_LEN;
	req_state->tag_ptr = req_state->challenge + mac_tag_offset;

	#ifdef PROFILING_REQUEST
		// State: Finished setting up HMAC input/output buffer...
		PROFILING_SET_COUNTER_F(2);
	#endif

	// (3.1) Compute MAC-Tag over hmac_input_buf (|  reb_cnt  |  req_cnt  |  access_type  | randomness |) and store it in the challenge buffer at position mac_tag_offset
	//       NOTE: We do NOT overwrite the randomness while writing the HMAC value, because the randomness is stored in hmac_input_buf, whereas we write to req_state->challenge!
	hash_hmac(keys_struct->key_gw_s, hmac_input_buf, HMAC_KEY_LEN, sizeof(hmac_input_buf), req_state->challenge + mac_tag_offset);

	#ifdef PROFILING_REQUEST
		// State: Finished HMAC computation...
		PROFILING_SET_COUNTER_F(3);
	#endif

	// (3.2) Copy the first three fields (reb_cnt, req_cnt, access_type) from the hmac_input_buf into the challenge buffer
	memcpy(req_state->challenge, hmac_input_buf, mac_tag_offset);

	// TODO: Figure out if we should add some randomness to prevent message from being used in potentially another protocol (cryptographic hygiene).
	//		 Gene suggested that the server should add some randomness to its authentication response, so it'd make sense also for the gateway to do it.

	// (4) Increment request counter value
	cntrs_struct->req_cnt++;

	// (5) Set receive time
	req_state->timer_set_time = HAL_GetTick();

	// (6) Set new request state to true (i.e. valid)
	req_state->valid = TRUE;



	return TRUE;
}

err_t proc_check_response(msg_t* msg_state, req_t* req_state, crypto_context_t* keys_struct){

	// (1) Check if the current request is valid, i.e. a valid candidate for being granted
	if(!req_state->valid){
		return -1;
	}

	// (2) Check if request has timed out yet
	if(HAL_GetTick() - req_state->timer_set_time > req_state->timeout){
		req_state->valid = FALSE;
		return -2;
	}

	#ifdef PROFILING_RESPONSE
		// State: Finished performing preliminary checks: whether we currently have a pending request, and if so, whether it has not timed out yet
		PROFILING_SET_COUNTER_F(1);
	#endif

	// (3) Verify message

	// (3.1) Generate message buffer with same format the server used, i.e.: |  randomness  |  request_mac_tag  |
	byte_t hmac_input_buf[RANDOM_LEN + HMAC_OUTPUT_SIZE];
	memcpy(hmac_input_buf, msg_state->p_payload, RANDOM_LEN);
	memcpy(hmac_input_buf + RANDOM_LEN, req_state->tag_ptr, HMAC_OUTPUT_SIZE);

	// (3.2) Compute reference tag over that buffer
	byte_t reference_tag[HMAC_OUTPUT_SIZE];

	#ifdef PROFILING_RESPONSE
		// State: Finished setting up HMAC input/output buffers
		PROFILING_SET_COUNTER_F(2);
	#endif

	hash_hmac(keys_struct->key_s_gw, hmac_input_buf, HMAC_KEY_LEN, RANDOM_LEN + HMAC_OUTPUT_SIZE, reference_tag);

	#ifdef PROFILING_RESPONSE
		// State: Finished computing HMAC
		PROFILING_SET_COUNTER_F(3);
	#endif

	// (3.3) Extract tag from server's response message
	byte_t* received_tag = msg_state->p_payload + RANDOM_LEN;

	// (3.4) Compare locally computed reference HMAC tag with that extracted from the payload. This is NOT constant time! If the match, we return 0 (no error), else -3 (HMAC verifiaction error).
	int difference = memcmp(reference_tag, received_tag, HMAC_OUTPUT_SIZE);
	if(difference == 0){

		#ifdef PROFILING_RESPONSE
			// State: Compared reference tag with that in the message
			PROFILING_SET_COUNTER_F(4);
		#endif

		// (3.5) Copy randomness into the request state, it will be used in HMAC computation of the next request by the GW
		memcpy(req_state->randomness, msg_state->p_payload, RANDOM_LEN);


		return 0;
	}
	else{
		return -3;
	}
}
