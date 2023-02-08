/*
 * pairing_task.c
 *
 *  Created on: Sep 12, 2022
 *      Author: SA4P Authors
 */

#include "compact25519.h"
#include "pairing_task.h"

static uint8_t psk[KEY_LEN] __attribute__ ((aligned (4))) = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
};

static uint8_t s_priv[KEY_LEN] __attribute__ ((aligned (4))) =
	{ 0xe8, 0x6d, 0x38, 0x2d, 0x23, 0xbb, 0x8b, 0x6c, 0x3, 0x8a, 0x40,
	  0xec, 0x68, 0x9, 0x16, 0x9d, 0x15, 0xb8, 0x43, 0xa6, 0x3d, 0x3c,
	  0x93, 0xab, 0xdb, 0xff, 0x35, 0x53, 0x92, 0x2c, 0x84, 0x7f };

static uint8_t s_pub[KEY_LEN] __attribute__ ((aligned (4))) =
	{ 0x50, 0xd2, 0x81, 0x3e, 0x76, 0x11, 0xfe, 0x1, 0x77, 0x42, 0x13,
	  0x85, 0xe1, 0x93, 0xde, 0x1, 0x7f, 0x22, 0x59, 0xa2, 0x5c, 0x27,
	  0x86, 0x45, 0xe3, 0xed, 0x74, 0xf7, 0x23, 0x80, 0x83, 0x70 };


static byte_t e_gw_priv[KEY_LEN], e_gw_pub[KEY_LEN];

//static uint8_t session_key_su[PAIRING_HANDSHAKE_KEY_SIZE], session_key_us[PAIRING_HANDSHAKE_KEY_SIZE];
//static uint32_t has_active_session_key = 0;


// NOTE: KEY_LEN == KEY_LEN!

void pairing_create_ephemeral_keypair(byte_t o_priv[KEY_LEN], byte_t o_pub[KEY_LEN]){
	byte_t rand_buf[KEY_LEN];
	rng_get_randomness(rand_buf, KEY_LEN);
	compact_x25519_keygen(o_priv, o_pub, rand_buf);
}

void pairing_derive_session_keys(byte_t* e_s_pub){

	// (1) Produce inputs into HKDF
	byte_t ikm[3 * KEY_LEN];
	compact_x25519_shared(ikm, s_priv, e_s_pub);
	compact_x25519_shared(ikm + KEY_LEN, e_gw_priv, e_s_pub);
	memcpy(ikm + 2*KEY_LEN, psk, KEY_LEN);

	#ifdef PROFILING_SIGNUP_SECOND_MESSAGE
			// 5: Performed two DH operations
			PROFILING_SET_COUNTER_F(5);
	#endif

	byte_t* infos[2] = {"gw_s", "s_gw"};

	byte_t  salt[16];
	memset(salt, 0, sizeof(salt));

	byte_t* o_bufs[2] = {cc_struct.key_gw_s, cc_struct.key_s_gw};

	#ifdef PROFILING_SIGNUP_SECOND_MESSAGE
			// 6: Prepared HKDP data
			PROFILING_SET_COUNTER_F(6);
	#endif

	// (2) Call HKDF to create two KEY_LEN long keys, the MAC-Key for gw->s communication and for s->gw communication
	hash_hkdf(ikm, salt, infos, 3*KEY_LEN, sizeof(salt), 4, 2, o_bufs);

	#ifdef PROFILING_SIGNUP_SECOND_MESSAGE
			// 7: Computed final session keys using HKDF
			PROFILING_SET_COUNTER_F(7);
	#endif
}


void pairing_create_first_message(byte_t* o_buf){
	// Note: This does NOT create the full signup request message, only the pairing part, i.e. the keys (and not the header and device type)

	// (1) Copy static public key into first 32 bytes of output buffer
	memcpy(o_buf, s_pub, KEY_LEN);

	#ifdef PROFILING_SIGNUP_FIRST_MESSAGE
		//			PROFILING_SET_PIN(gpio_profiling_pin_2);
		// 2: Copied 32 byte public key into output buffer (i.e. payload buffer)
		PROFILING_SET_COUNTER_F(2);
	#endif

	// (2) Create ephemeral keypair and store it in the buffers e_gw_priv, e_gw_pub above, allocated at compiletime.
	pairing_create_ephemeral_keypair(e_gw_priv, e_gw_pub);

	#ifdef PROFILING_SIGNUP_FIRST_MESSAGE
		//			PROFILING_SET_PIN(gpio_profiling_pin_2);
		// 3: Created ephemeral keypair
		PROFILING_SET_COUNTER_F(3);
	#endif

	// (3) Copy the public ephemeral key into the second 32 bytes of the output buffer
	memcpy(o_buf + KEY_LEN, e_gw_pub, KEY_LEN);

	// (4) Compute HMAC(psk, s_pub || e_gw_pub) and store it into the third (resp. last) 32 bytes of the output buffer

	// (4.1) Concatenate s_pub and e_gw_pub into one buffer

	#ifdef PROFILING_SIGNUP_FIRST_MESSAGE
		//			PROFILING_SET_PIN(gpio_profiling_pin_2);
		// 4: Moved data around
		PROFILING_SET_COUNTER_F(4);
	#endif

	byte_t hmac_input[2 * KEY_LEN];
	memcpy(hmac_input, s_pub, KEY_LEN);
	memcpy(hmac_input + KEY_LEN, e_gw_pub, KEY_LEN);

	#ifdef PROFILING_SIGNUP_FIRST_MESSAGE
		//			PROFILING_SET_PIN(gpio_profiling_pin_2);
		// 5: Prepared HMAC input buffer
		PROFILING_SET_COUNTER_F(5);
	#endif

	// (4.2) Actually compute HMAC
	hash_hmac(psk, hmac_input, KEY_LEN, KEY_LEN, o_buf + 2 * KEY_LEN);

	#ifdef PROFILING_SIGNUP_FIRST_MESSAGE
		//			PROFILING_SET_PIN(gpio_profiling_pin_2);
		// 6: Computed HMAC
		PROFILING_SET_COUNTER_F(6);
	#endif

}


void pairing_process_second_message(byte_t* i_buf){
	// (1) Extract server's ephemeral public key E_S and the MAC-Tag = MAC_{psk}(E_S || e_pub)
	byte_t* e_s_pub = i_buf;
	byte_t* tag     = i_buf + KEY_LEN;

	#ifdef PROFILING_SIGNUP_SECOND_MESSAGE
			// 1: Retrieved server's ephmeral public key and MAC tag
			PROFILING_SET_COUNTER_F(1);
	#endif

	// (2) Check MAC-Tag
	byte_t reference_tag[HMAC_OUTPUT_SIZE];
	byte_t hmac_input[2 * KEY_LEN];
	memcpy(hmac_input,           e_s_pub, KEY_LEN);
	memcpy(hmac_input + KEY_LEN, e_gw_pub, KEY_LEN);

	#ifdef PROFILING_SIGNUP_SECOND_MESSAGE
			// 2: Prepared HMAC input buffer
			PROFILING_SET_COUNTER_F(2);
	#endif

	hash_hmac(psk, hmac_input, KEY_LEN, sizeof(hmac_input), reference_tag);

	#ifdef PROFILING_SIGNUP_SECOND_MESSAGE
			// 3: Computed HMAC
			PROFILING_SET_COUNTER_F(3);
	#endif
	uint8_t diffs = memcmp(tag, reference_tag, HMAC_OUTPUT_SIZE);

	if(diffs != 0){
		// Case: Invalid MAC-Tag
		while(TRUE){
			// For testing purposes, we just enter an infinite loop
			__NOP();
		}
	}

	#ifdef PROFILING_SIGNUP_SECOND_MESSAGE
			// 4: Compared HMAC
			PROFILING_SET_COUNTER_F(4);
	#endif

	// If we reached here, the MAC-Tag was valid

	// (3) Compute session keys
	pairing_derive_session_keys(e_s_pub);


}
