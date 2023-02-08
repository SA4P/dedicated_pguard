/*
 * recv_task.c
 *
 *  Created on: Apr 2, 2022
 *      Author: SA4P Authors
 */
#include "recv_task.h"

int test_cnt = 0;

typedef enum{
	STATE_GET_PAYLOAD_TYPE	= 0,
	STATE_GET_PAYLOAD_LEN	= 1,
	STATE_GET_PAYLOAD		= 2
} state_t;

void task_receive(msg_t* msg_state){
	static state_t state = STATE_GET_PAYLOAD_TYPE;

	bool_t valid;														// Set to 1 if value is in permissible range

	if(state == STATE_GET_PAYLOAD_TYPE){

		byte_t b;
		if(rb_consume_one(&rx_rbuf, &b)){
			valid = rx_check_type(b);
			if(valid){
				msg_state->payload_type = b;
				state = STATE_GET_PAYLOAD_LEN;
			}
		}
		else{
			return;
		}
	}

	if(state == STATE_GET_PAYLOAD_LEN){
		uint16_t len;

		if(rb_consume(&rx_rbuf, &len, 2)){														// Try to consume 2 bytes from receive buffer
			valid = rx_check_len(len, msg_state->payload_type);
			if(valid){																			// Check whether received length is valid
				msg_state->payload_len = len;
				state = STATE_GET_PAYLOAD;
			}
			else{																				// Case: Received length invalid ==> Discard and start anew
				// TODO: Somehow notify for resynchronization and debug purposes!
				state = STATE_GET_PAYLOAD_TYPE;
			}
		}
	}

	if(state == STATE_GET_PAYLOAD){
		uint16_t len = msg_state->payload_len;
		if(rb_consume(&rx_rbuf, msg_state->p_payload, len)){
			state = STATE_GET_PAYLOAD_TYPE;
			flag_msg_rdy = TRUE;
			test_cnt++;
		}
	}
}
