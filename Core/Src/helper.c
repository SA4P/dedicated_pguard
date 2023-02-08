/*
 * helper.c
 *
 *  Created on: Mar 28, 2022
 *      Author: SA4P Authors
 */

#include "helper.h"

// - - - - - - - - - - - - - - -
// RCC
// - - - - - - - - - - - - - - -


// Enables GPIO port G wrt. RCC (i.e. such that GPIO port G is functional)
void rcc_enable_gpiog(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
}

void rcc_enable_gpiod(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
}

void rcc_enable_gpioc(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
}

void rcc_enable_gpioe(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
}

void rcc_enable_gpiof(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
}

// Enables LPUART1 wrt. RCC (i.e. such that LPUART1 is functional. Otherwise, writing to/ reading from the peripheral would not even reach the peripheral)
void rcc_enable_lpuart(){
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
}

// Enables clock source of USART2, such that it can be programmed and interacted with
void rcc_enable_usart2(){
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART2EN);
}

void rcc_set_vddio2(){
	PWR->CR2 = PWR_CR2_IOSV;
}

// - - - - - - - - - - - - - - -
// GPIO
// - - - - - - - - - - - - - - -
void gpio_init_uart(USART_TypeDef * uart){
	GPIO_TypeDef * gpio;
	if (uart == LPUART1){										// Case: Configure GPIO pins for LPUART1 ==> Pins PG7 and PG8 (i.e. 92, 93) in our case

		/*
		 * We operate on pins Pins PG7 and PG8 (i.e. 92, 93). Hence, we first set gpio to point to the corresponding registers
		 * OSPEEDR: Left at reset, meaning low speed.
		 * PUPDR:   Left at reset because we want the pin to neither be pull-up or pull-down.
		 * AFRL:    Here this is AFR[0]. Controls the alternate function of pins PG0 to PG7 (inclusive). Each pin gets 4 bits of the reg (pin i has bits (i*4+3, i*4+2, i*4+1, i*4)).
		 * AFRH:    Here this is AFR[1]. Controls the alternate function of pins PG8 to PG15 (inclusive).
		 * MODER:   Controls whether a pin is in input mode (0b00), gen-purp. output mode (0b01), alternate function mode (0b10) or analog mode (reset state) (0b11). By default, each pin has MODER value 0b11 (IN PORT G!!! DOES NOT HOLD FOR PORTS A, B, H).
		 */

		gpio = GPIOG;

		gpio->AFR[0] |= GPIO_AF8_LPUART1 << 28;						// GPIO_AF8_LPUART1 has value 0b00001000, which in turn is shifted by 28 bits such that the bit at position 3 (starting at 0) arrives at bit 31, the MSB of AFRL which corresponds to the 3rd bit (starting at 0) of PG7. This is the pin we need to set to 1
		gpio->AFR[1] |= GPIO_AF8_LPUART1;							// PG8's alternative function is set using the 4LSB of AFRH, which is indexed by AFR[1] (I think). Hence, we can just set AFR[1] to GPIO_AF8_LPUART1

		// ASSUMPTION: MODER[14] == MODER[16] == 0
		gpio->MODER ^= (0x1 << 14 | 0x01 << 16);	// MODER is by default set to 0b1111...1 (i.e. all 1 bits). We want the 7th and 8th pin (with bit fields 15:14, 17:16 resp.) to have value 0b10 (Alternative function mode)
																// ==> xor MODER's 14th and 16th bit with 0x01 to get the 14th and 16th bits of MODER to be 0

//		gpio->MODER |=

	}
	else if(uart == USART2){											// Case: Configure GPIO pins for USART2 ==> Pins PD5 and PD6 (i.e. 119, 122) in our case

		gpio = GPIOD;

		gpio->AFR[0] |= GPIO_AF7_USART2 << 20 | GPIO_AF7_USART2 << 24;	// AF7 is the alternative function description of USART2 for the GPIO pins PD5, PD6

		// ASSUMPTION: MODER[10] == MODERL[12] == 0
		gpio->MODER  ^= 0x1 << 10 | 0x1 << 12;							// Change the mode selection bits 0b11, 0b11 of PD5, PD6 to 0b10, 0b10 .

	}
	else{
		// NOTE: We do not plan to need more than 2 (LP)U(S)ARTs!
		while(TRUE){
			__NOP();
		}
	}
}

void gpio_init_button(){
	GPIO_TypeDef * gpio = GPIOC;

	// (1) Set pin mode to input (MODE7 == 0b00)
	CLEAR_BIT(gpio->MODER, GPIO_MODER_MODE13);

	// (2) Activate pull-down resistor (PUPD7 == 0b10)
	SET_BIT(gpio->PUPDR, GPIO_PUPDR_PUPD13_1);
}

// Sets ALL pins of the GPIO port gpio to output, i.e. all 16 pins
void gpio_init_port_as_output(GPIO_TypeDef * gpio){

	// (1) Set pin mode to input (MODE7 == 0b00)
	uint32_t bits_to_clear = GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1 |
			GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1 |
			GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1;

	CLEAR_BIT(gpio->MODER, bits_to_clear);

	// (2) Activate pull-down resistor (PUPD7 == 0b10)
//	SET_BIT(gpio->PUPDR,);
}

// - - - - - - - - - - - - - - -
// DEBUGGING
// - - - - - - - - - - - - - - -

void debug_gpio_set_g7_output(){
	GPIOG->MODER ^= 0x01 << 15;										// Set mode register bits associated with GPIO G7 to 0b01 (rather than the default 0b11)
}


// - - - - - - - - - - - - - - -
// USART
// - - - - - - - - - - - - - - -
void usart_init_cr1(USART_TypeDef * uart){
	uart->CR1 = 0x0;												// Initialize to no interrupts, no nothing
//	uart->CR1 = 0b111 << 5;											// Sets TXEIE, TCIE, RXNEIE to 1 (enables them) in CR1
}

void usart_init_cr2(USART_TypeDef * uart){
	// So far, it doesn't seem as if we need to change CR2 as we don't use any of the features it controls.
}

void usart_init_cr3(USART_TypeDef * uart){
	uart->CR3 = 0b01;												// Sets EIE to 1, which means errors create an interrupt
}

void usart_init_baudrate(USART_TypeDef * uart, uint32_t baudrate){

	if (uart == LPUART1){
		uint64_t usartdiv;
		usartdiv = 256 * PCLK1_FREQ;							// For LPUART, page 1658-1659 of reference manual state how to set usartdiv.
																// Note that lpuart_kec_ck = lpuart_pclk and that we have divider register set to 0 (i.e. no divide),
																// which means lpuart_ckpres = lpuart_pclk = PCLK1_FREQ.
																// ==> To get Baud rate 115200, we need to set BRR (i.e. LPUARTDIV) such that 256 * PCLK1_FREQ / LPUARTDIV = 115200 (or close to it)
																// Equality holds <==> LPUARTDIV = 256 * PCLK1_FREQ / 115200.
																// NOTE: Computation would be OVERFLOW if we didn't cast PCLK1_FREQ in its definition to be uint64_t!
		usartdiv = usartdiv / baudrate;
		uart->BRR = USART_BRR_LPUART & usartdiv;				// LPUART is 20 bits, so just to be super formal = ANDded it with USART_BRR_LPUART, which is 0xFFFFF, i.e. 20 LSB set to 1

	}
	else{
		uint16_t usartdiv;
		usartdiv = PCLK1_FREQ / baudrate;						// 110 * 10^6 / 115200 = 954.8611 = 954 per integer division
		uart->BRR = USART_BRR_BRR & usartdiv;										// usartdiv is already a 16 bit value and the 16 LSB of BRR are meant to hold the usartdiv value. Nevertheless, I add it with USART_BRR_BRR == 0xFFFF just to be super formal

	}

}

void usart_enable(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_UE;							// Set UE bit to 1 by ORing it with 0x01. This sets is to 1 if it wasn't 1 already
}

void usart_enable_receive(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_RE;							// Set UE bit to 1 by ORing it with 0x01. This sets is to 1 if it wasn't 1 already
}

void usart_enable_transmit(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_TE;							// Set UE bit to 1 by ORing it with 0x01. This sets is to 1 if it wasn't 1 already
}

void usart_set_TXEIE(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_TXEIE;						// Set TXEIE bit to 1 ==> get interrupted upon TXE flag being set (TDR being empty)
}

void usart_set_RXNEIE(USART_TypeDef * uart){
	uart->CR1 = uart->CR1 | USART_CR1_RXNEIE;						// Set RXNEIE bit to 1 ==> get interrupted upon RXNE flag being set (RDR being not empty)
}


// - - - - - - - - - - - - - - -
// RING BUFFER
// - - - - - - - - - - - - - - -

void rb_init(ringbuf_t * rb, char init_val){
	  rb->cons_ind	= 0;
	  rb->prod_ind  = 0;
	  rb->empty		= TRUE;
	  rb->isr_ctr   = 0;
	  memset(rb->buf, init_val, sizeof(rb->buf));
}

bool_t rb_consume_one(ringbuf_t* rb, byte_t* obyte){
	int16_t prod_ind 	 = rb->prod_ind;
	int16_t cons_ind	 = rb->cons_ind;
	int16_t buffsize 	 =	RBUF_LEN;



	if(prod_ind == cons_ind){			// When buffer is empty, i.e.
		return FALSE;
	}

	*obyte = rb->buf[cons_ind];

	int16_t new_cons_ind = mod(cons_ind + 1, buffsize);			// NOTE: cons_ind trails one behind, i.e. when cons_ind == i, then i has already been read!
	rb->cons_ind		 = new_cons_ind;

	return TRUE;

}

int16_t rb_consume_all(ringbuf_t* rb, byte_t* obuf){
	int16_t prod_ind 	 = rb->prod_ind;
	int16_t cons_ind	 = rb->cons_ind;
	int16_t buffsize 	 =	RBUF_LEN;

	int16_t num_readable = mod(prod_ind - cons_ind, buffsize);
	rb_consume(rb, obuf, num_readable);

	return num_readable;
}


bool_t rb_consume(ringbuf_t* rb, byte_t * obuf, int16_t num_items){
	int16_t prod_ind	= rb->prod_ind;
	int16_t cons_ind 	= rb->cons_ind;
	int16_t buffsize 	= RBUF_LEN;
	byte_t*  rbuf      	= rb->buf;

	int16_t num_readable = mod(prod_ind - cons_ind, buffsize);

	if(num_items > num_readable){			// Case: Consume pointer would overtake produce pointer
		return FALSE;
	}

	for(int i = 0; i < num_items; ++i){
		obuf[i] = rbuf[mod(cons_ind + i, buffsize)];
	}
	rb->cons_ind = mod(cons_ind + num_items, buffsize);							// NOTE: We have to do it like this because having "rb->cons_ind = mod(cons_ind + i, buffsize);" in the loop undercounts by one (the i = 0 case adds 0 to rb->cons_ind)
	return TRUE;
}

bool_t rb_produce_one(ringbuf_t* rb, byte_t ibyte){

	return rb_produce(rb, &ibyte, 1);
//	int16_t prod_ind = rb->prod_ind;
//	int16_t cons_ind = rb->cons_ind;
//	int16_t buffsize 	=	RBUF_LEN;
//
//	if(mod(prod_ind + 1 - cons_ind, buffsize) == 0){										// Buffer of n elements is filled with n-1 elements. We don't allow the n-th element to be populated
//																				// as we would otherwise get into the ambiguous case where prod_ind == cons_ind, which we DEFINE to mean empty (and allowing n-th element to be placed would make the full buffer appear as if it was empty)
//		return FALSE;
//	}
//
//	rb->buf[prod_ind] = ibyte;
//	rb->prod_ind = mod(prod_ind + 1, buffsize);
//
//	return TRUE;
}

bool_t rb_produce(ringbuf_t* rb, byte_t* ibuf, int16_t num_items){
	int16_t prod_ind 	= rb->prod_ind;
	int16_t cons_ind 	= rb->cons_ind;
	int16_t buffsize 	= RBUF_LEN;
	byte_t*  rbuf      	= rb->buf;


	if(mod(prod_ind + num_items - cons_ind, buffsize) == 0 || num_items >= buffsize){
		return FALSE;
	}


	for(int i = 0; i < num_items; ++i){
		rbuf[mod(prod_ind + i, buffsize)] = ibuf[i];				// We take local copy of cons_ind for efficiency ==> add + i
	}

	rb->prod_ind = mod(prod_ind + num_items, buffsize);					// Update volatile, global cons_ind AFTER consuming to prevent buffer value from being overwritten

	return TRUE;

}

// Checks for a byte if it is a valid payload type byte
bool_t rx_check_type(byte_t b){
	if(PAYLOAD_SIGNUP <= b && b <= PAYLOAD_SIGNUP_RESP){
		return TRUE;
	}
	return FALSE;
}

bool_t rx_check_len(uint16_t len, byte_t b){
	switch (b) {
		case PAYLOAD_REQUEST:
			__NOP();
			return len == LEN_PAYLOAD_REQUEST;
			break;
		case PAYLOAD_RESPONSE:
			return len == LEN_PAYLOAD_RESPONSE;
			break;
		case PAYLOAD_SIGNUP_RESP:
			return len == LEN_PAYLOAD_SIGNUP_RESP;
			break;
		default:
			return FALSE;
			break;
	}

}

// - - - - - - - - - - - - - - -
// RNG
// - - - - - - - - - - - - - - -

void rng_wait_ready(){
	while(!READ_BIT(RNG->SR, RNG_SR_DRDY)){
		__NOP();
	}
}

void rng_init(){

	// (1) Clock configuration

	// Switch on HSI48 48MHz oscillator
	RCC->CRRCR |= RCC_CRRCR_HSI48ON;

	// Wait until it is stable by polling on HSI48RDY bit of RCC->CRRCR (Sec. 9.8.31, page 404/22194 of reference manual RM0438)
	while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY_Msk)){
		__NOP();
	}

	// Enable RNG peripheral (wrt. RCC ==> Give it "juice")
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;

	// 48 MHz clock source selector set to HSI48 (the above only enabled HSI48, we still need to set a downstream MUX to have the HSI48 delivered to RNG
	RCC->CCIPR1 = RCC->CCIPR1^ (RCC->CCIPR1 & RCC_CCIPR1_CLK48MSEL);						// Idea: We XOR the bits 27:26 (responsible for setting CLK48MSEL) with itself, guarantees that they are 0 afterwards. All the other bits of (RCC->CCIPR1 & RCC_CCIPR1_CLK48MSEL) are 0 ==> No other bits affected

	// (2) RNG configuration

	// Disable RNG. This is "just in case" as it should be disabled at start
	CLEAR_BIT(RNG->CR, RNG_CR_RNGEN);

	// Build our configuration values:
	uint32_t config_vals = RNG_CR_CED | RNG_CR_CONDRST;

	// Set our config values
	SET_BIT(RNG->CR, config_vals);

	// Set self check values
	WRITE_REG(RNG->HTCR, RNG_HTCFG_1);
	WRITE_REG(RNG->HTCR, RNG_HTCFG);

	// Finishing the setup by clearing the conditional soft reset bit again (like a transaction: Setting it to 1 started the transaction, now we end it)
	CLEAR_BIT(RNG->CR, RNG_CR_CONDRST);

	// Wait until the conditional configuration has completed:
	while(READ_BIT(RNG->CR, RNG_CR_CONDRST)){					// NOTE: Bit set to 1 means reset not complete ==> Wait until bit is 0
		__NOP();
	}

	// Enable RNG peripheral
	SET_BIT(RNG->CR, RNG_CR_RNGEN);

	// Check for seed error. This is done by checking the Seed error INTERRUPT flag, which is set even if interrupts are disabled (if I understood correctly)
	if(READ_BIT(RNG->SR, RNG_SR_SEIS)){
		while(TRUE){
			__NOP();
		}
	}

	// Wait until the first batch of 4 x 32 bytes of randomness are ready
	while(!READ_BIT(RNG->SR, RNG_SR_DRDY)){
		__NOP();
	}
}

// obuf[0], obuf[1], ..., obuf[num_samples - 1] get populated with random bytes
// NOTE: We do not check for any kind of errors in the randomness sampling!
void rng_get_randomness(byte_t* o_buf, uint32_t num_samples){
	uint32_t rand_val;

	int remainder = (int) (num_samples % 4);
	int32_t full_block_indices = num_samples - remainder;

	for(int i = 0; i < full_block_indices; i += 4){
		rng_wait_ready();
		rand_val = READ_REG(RNG->DR);
//		if(rand_val == 0){										// Check whether randomness generation has failed. As stated in the reference manual, one sign of problem is an all-0 output (also incredibly unlikely!)
//			while(TRUE){										// This check is BAD, as it can by chance happen that we get all-0, which would lock up the system!
//				__NOP();
//			}
//		}
		memcpy(o_buf + i, &rand_val, 4);
	}

	// If message buffer is not multiple of 4 bytes in size, we handle the last <= bytes separately
	if(remainder > 0){
		rng_wait_ready();
		rand_val = READ_REG(RNG->DR);
		memcpy(o_buf + full_block_indices, &rand_val, 4);
	}

}

// - - - - - - - - - - - - - - -
// HASH
// - - - - - - - - - - - - - - -


void hash_init(){
	SET_BIT(HASH->CR, HASH_CR_INIT);
}

// Waits until DIN and the FIFO behind it are empty ==> Can load a new block
void hash_wait_input_empty(){
	while(!READ_BIT(HASH->SR, HASH_SR_DINIS)){
		__NOP();
	}
}

// Waits until DIN and the FIFO behind it are empty ==> Can load a new block
void hash_wait_finished(){
	while(!READ_BIT(HASH->SR, HASH_SR_DCIS)){
		__NOP();
	}
}

// Extracted almost one-to-one from the HAL library! The first line seems sketchy but it works in the HAL library...
// NOTE: Casting a pointer to an int certainly doesn't work on all (e.g. 64-bit) platforms!
void hash_extract_digest(byte_t* o_buf){
	uint32_t msgdigest = (uint32_t)o_buf;

	*(uint32_t*)(msgdigest) = __REV(HASH->HR[0]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH->HR[1]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH->HR[2]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH->HR[3]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH->HR[4]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH_DIGEST->HR[5]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH_DIGEST->HR[6]);
	msgdigest+=4U;
	*(uint32_t*)(msgdigest) = __REV(HASH_DIGEST->HR[7]);
}

void hash_startup(){
	// (1) Enable clock to it

	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_HASHEN);

	// Added those two lines because HAL also has it, I think it's to delay the execution until we can be sure the bit has been written accordingly
    uint32_t tmpreg = READ_BIT(RCC->AHB2ENR, RCC_AHB2ENR_HASHEN);
    UNUSED(tmpreg);

	// (2) Set up HASH peripheral

    // Set all values to reset
    CLEAR_REG(HASH->CR);

    // Set datatype to bytes ==> handles endianness issue
    SET_BIT(HASH->CR, HASH_CR_DATATYPE_1);

    // Set HASH mode to HMAC ==> Knows how to handle key, etc.
    SET_BIT(HASH->CR, HASH_CR_MODE);

    // Set hashing algorithm to have control bits set to 11 ==> SHA256
    SET_BIT(HASH->CR, HASH_CR_ALGO_1 | HASH_CR_ALGO_0);

    // Toggle INIT bit ==> Reset the peripheral's input buffer AND select the hashing that has been set in the command above.
    hash_init();
}

// Updates the hash state with data of length AT MOST ONE BLOCK (512 bits == 64 bytes).
void hash_update(byte_t *block_buf, uint32_t len_bytes){

	// Check whether input is actually at most 64 bytes (1 block) in length
	if(len_bytes > 64){
		while(TRUE){
			__NOP();
		}
	}

	uint32_t interm_buf;															// I THINK this can prevent misaligned memory accesses
	for(int i = 0; i < len_bytes; i += 4){

		// Copy buffer bytewise into a 4-byte aligned buffer
		memcpy(&interm_buf, block_buf + i, 4);

		// Write content of 4-byte aligned buffer into HASH's DIN register
		WRITE_REG(HASH->DIN, interm_buf);

		// XXX: It seems as if we don't need to wait as we are inserting
		//      data of at most one block in size and the hash peripheral can handle that
	}
}

// Digests a WHOLE message, i.e. also some of length > 1 block!
// Does NOT call init.
void hash_prepare_digest(byte_t* msg_buf, uint32_t len_bytes){
	uint8_t suffix_len = 8 * (len_bytes % 4);										// HASH peripheral works on 32 bit words
																					// ==> If we have e.g. 24 bits of data, the upper 8 bits would be 0
	MODIFY_REG(HASH->STR, HASH_STR_NBLW_Msk, suffix_len);

	for(int i = 0; i < len_bytes; i += 64){
		// Load new block into HASH peripheral
		hash_update(msg_buf + i, min(len_bytes - i, 64));				// We call the hash_update function with either a full block (64 bytes) or with length suffix_len

		// Wait until the HASH peripheral can take in a new BLOCK of data
//		hash_wait_input_empty();
	}
}

void hash_hmac(byte_t* key_buf, byte_t* msg_buf, uint32_t len_key, uint32_t len_msg, byte_t* o_buf){
	// Step 1: Initialize peripheral
	hash_init();																	// Resets the data FIFO within HASH peripheral, BUT KEEPS THE CONFIGS otherwise

	// Step 2.0: Insert inner key (sets NBLW internally aswell)
	hash_prepare_digest(key_buf, len_key);

	// Step 2.1: Start digest
	SET_BIT(HASH->STR, HASH_STR_DCAL);

	// Step 2.2: Wait until input register and fifo are empty again
	hash_wait_input_empty();

	// Step 3.0: Insert message (sets NBLW internally aswell)
	hash_prepare_digest(msg_buf, len_msg);

	// Step 3.1: Start digest
	SET_BIT(HASH->STR, HASH_STR_DCAL);

	// Step 3.2: Wait until input register and fifo are empty again
	hash_wait_input_empty();

	// Step 4.0: Insert outer key (sets NBLW internally aswell)
	hash_prepare_digest(key_buf, len_key);

	// Step 4.1: Start digest
	SET_BIT(HASH->STR, HASH_STR_DCAL);

	// Step 4.2: Wait until HASH computation is done, s.t. we can get the output
	hash_wait_finished();

	hash_extract_digest(o_buf);

}

// o_buf holds PRK, the pseudorandom key
void hash_hkdf_extract(byte_t* salt, byte_t* ikm, uint32_t salt_len, uint32_t ikm_len, byte_t* o_buf){
	hash_hmac(salt, ikm, salt_len, ikm_len, o_buf);
}

// This HKDF expand step is simplified, it only works for L == HMAC_OUTPUT_SIZE (see RFC 5869 for def'n of L).
// L < HMAC_OUTPUT_SIZE gives a buffer overflow of o_buf, L > HMAC_OUTPUT_SIZE is not implemented.
void hash_hkdf_expand_simplified(byte_t* prk, byte_t* info, uint32_t prk_len, uint32_t info_len, byte_t* o_buf){

	// (1) Compute input for T(1) in RFC 5869, for that we take the info string and concatenate 0x01 to it
	byte_t hmac_input[info_len + 1];
	memcpy(hmac_input, info, info_len);
	hmac_input[info_len] = 0x01;

	// (2) compute T(1) itself, write it directly to the output buffer
	hash_hmac(prk, hmac_input, prk_len, sizeof(hmac_input), o_buf);
}

// Creates one output key of length o_key_len for each info string, so infos_num many
void hash_hkdf(
		byte_t* ikm,
		byte_t* salt,
		byte_t** infos,
		uint32_t ikm_len,
		uint32_t salt_len,
		uint32_t info_len,
		uint32_t infos_num,
		byte_t** o_bufs){

	byte_t prk[HMAC_OUTPUT_SIZE];

	hash_hkdf_extract(salt, ikm, salt_len, ikm_len, prk);

	// We have info_num many infos, each of length info_len. E.g: info_num == 2, info_len == 4, infos = "gw_ss_gw"
	//																						    info 0 ---^   ^--- info 1
	// We thus compute 2 output keys, each of length HMAC_OUTPUT_SIZE (per the restriction of our simplified hkdf expand), key 0 stored in o_bufs[0], key 1 in o_bufs[1]
	for(int i = 0; i < infos_num; ++i){
		hash_hkdf_expand_simplified(prk, infos[i], HMAC_OUTPUT_SIZE, info_len, o_bufs[i]);
	}

}

// - - - - - - - - - - - - - - -
// Timer
// - - - - - - - - - - - - - - -


// Sets up timer t. NOTE: presc is supposed to be one less than the desired prescaler. I.e. presc = 0 means clock taken directly, presc = 1 means clock halved, presc = 2 means clock 1/3 speed, ...
void timer_init(TIM_TypeDef* t, uint16_t presc){


	// TODO: Ask Piet about those two lines. Good because they fully reset the timer, but BAD because they also write to reserved bits within the registers! Is there a better way than to mask the reserved regs away?

//	// Clear CR1 (reset it)
//	CLEAR_REG(t->CR1);
//
//	// Clear CR2 (reset it)
//	CLEAR_REG(t->CR2);


	// Enable timer's clock input
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_TIM7EN);

	// Set prescaler
	WRITE_REG(t->PSC, presc);

	// Set repetition counter to 0
	CLEAR_REG(t->RCR);

// Do NOT set one-shot mode! It is not compatible with how we do the timer resetting in case an access needs to be extended in time!
//	SET_BIT(t->CR1, TIM_CR1_OPM);

}

// Starts a timer which goes off after timeout. NOTE that this depends on the prescaler! E.g. we use 10999, i.e. effectively clock divided by 11000. Hence, at 110MHz, this means the timer is incremented every 100us.
// E.g. for timout == 10, the timer overflows and causes update event interrupt after 10 * 100us = 1ms.
void timer_start(TIM_TypeDef* t, uint16_t timeout){

	if(t != TIM7){
		while(TRUE){
			__NOP();
		}
	}

	SET_BIT(DBGMCU->APB1FZR1, DBGMCU_APB1FZR1_DBG_TIM7_STOP);

	// Check if counter is currenty operating. This is an ERROR because we should NEVER restart a counter before it is finished!
	if(READ_BIT(t->CR1, TIM_CR1_CEN)){
		while(TRUE){
			__NOP();
		}
	}

	// Clear timer count register (16 least significant bits)
	CLEAR_BIT(t->CNT, 0xFFFF);

	// Set auto-reload register (value after which the counter overruns)
	// TODO: Find out whether it should actually be "timeout_100us - 1" because the datasheet says it counts "from 0 to the auto-reload value" in section 33.3.2, page 1080
	WRITE_REG(t->ARR, timeout);

	// NOTE: Prescaler Register is ALWAYS buffered ==> Need to commit the values into the prescaler shadow register by triggering an update event manually!
	SET_BIT(t->EGR, TIM_EGR_UG);
	CLEAR_BIT(t->SR, TIM_SR_UIF);

	//Start timer
	SET_BIT(t->CR1, TIM_CR1_CEN);


	// Set update interrupt to be enabled
	SET_BIT(t->DIER, TIM_DIER_UIE);

	// Enable global interrupt for that timer
	// Currently, only have timer TIM7!
	NVIC_EnableIRQ(TIM7_IRQn);

}

// - - - - - - - - - - - - - - -
// Flash
// - - - - - - - - - - - - - - -


void flash_set_waitstates(){
  // Set up Flash wait states given the system clock (Reference Manual RM0438, page 181, Section 6.3.3, Table 32)
  // (1) Set wait states
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_5WS);

  // (2) Wait until wait states set
  while(READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_5WS){
	  __NOP();
  }
}

void flash_erase_page_b1(uint8_t flash_target_page){


	while(flash_target_page >= FLASH_PAGE_NB_PER_BANK){
		__NOP();
	}
	FLASH_EraseInitTypeDef erase_init;
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.Banks 	   = FLASH_BANK_1;
	erase_init.Page	   = flash_target_page;		// We erase the last page of the bank
	erase_init.NbPages   = 1;

	uint32_t faulty_page;							// Page that caused HAL_FLASHEx_Erase to fail. Per default: 0xffffffff (2^32 - 1).
	HAL_FLASHEx_Erase(&erase_init, &faulty_page);
}

void flash_program_page_b1(uint8_t flash_target_page, uint16_t byte_offset, uint64_t* dword_input, uint32_t num_dwords){

	while(flash_target_page >= FLASH_PAGE_NB_PER_BANK || byte_offset + num_dwords * 8 > FLASH_PAGE_SIZE){
		__NOP();
	}

	HAL_StatusTypeDef hal_status;

	uint32_t flash_target_addr;

	for(int i = 0; i < num_dwords; i+=1){
		flash_target_addr = flash_get_addr(flash_target_page, i);
		hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_target_addr, dword_input[i]);
		while(hal_status != HAL_OK){
			__NOP();
		}
	}
}

void flash_read_b1(uint8_t flash_target_page, uint16_t byte_offset, uint64_t* o_buf, uint32_t num_dwords){

	while(flash_target_page >= FLASH_PAGE_NB_PER_BANK || byte_offset + num_dwords * 8 > FLASH_PAGE_SIZE){
		__NOP();
	}

	uint64_t* flash_target_addr = flash_get_addr(flash_target_page, byte_offset);

	for(int i = 0; i < num_dwords; ++i){
		o_buf[i] = flash_target_addr[i];
	}
}

uint64_t* flash_get_addr(uint8_t flash_target_page, uint16_t byte_offset){
	return FLASH_BASE_NS + flash_target_page * FLASH_PAGE_SIZE + byte_offset;
}

uint64_t* flash_get_page_addr(uint8_t flash_target_page){
	return flash_get_addr(flash_target_page, 0);
}


uint32_t reb_cnt_inc_and_get(){

//	 XXX: REMOVE THIS!! It is only here because debug mode seems to reboot the device intermittenly bef
	return 0;

	// (1) Load double-word reboot counter into reb_cnt_64 and store 32 bit version as reb_cnt (not necessary I think, but that's what made sense)
	uint64_t reb_cnt_64;
	flash_read_b1(FLASH_REB_CNT_PAGE, 0, &reb_cnt_64, 1);
	uint32_t reb_cnt = reb_cnt_64 & 0xffffffff;

	// (2) Load double-word reboot reference counter into reb_cnt_64 and store 32 bit version as reb_ref_cnt (not necessary I think, but that's what made sense)
	uint64_t reb_ref_cnt_64;
	flash_read_b1(FLASH_REB_REF_CNT_PAGE, 0, &reb_ref_cnt_64, 1);
	uint32_t reb_ref_cnt = reb_ref_cnt_64 & 0xffffffff;

	// (3) Check if they disagree
	if(reb_cnt != reb_ref_cnt){
		if(reb_cnt == reb_ref_cnt + 1){
			// Handled case: Reboot after reboot counter was incremented but before reference reboot counter was incremented
			// ==> reb_cnt already holds the new, larger value!
			reb_ref_cnt = reb_cnt;
			reb_ref_cnt_64 = reb_ref_cnt;
			flash_program_page_b1(FLASH_REB_REF_CNT_PAGE, 0, &reb_ref_cnt_64, 1);

			return reb_cnt;
		}
		else{
			// Unhandled case
			while(TRUE){
				__NOP();
			}
		}
	}

	// If we reach here, we know that reb_cnt == reb_ref_cnt, so we can increment both!

	// (4) Increment reboot counter and reboot reference counter and write back to flash
	//	   By our assumption that reboots cannot happen while writing flash, agreement means the counters are correct and that we can just use them
	//     (if rebooting during writing was possible, it could happen that this causes both counters to be corrupted and coincidentally to the same value)

	reb_cnt_64 = ++reb_cnt;
	reb_ref_cnt_64 = ++reb_ref_cnt;

	// (4.1) Write back the updated values

	flash_erase_page_b1(FLASH_REB_CNT_PAGE);
	flash_program_page_b1(FLASH_REB_CNT_PAGE, 0, &reb_cnt_64, 1);

	flash_erase_page_b1(FLASH_REB_REF_CNT_PAGE);
	flash_program_page_b1(FLASH_REB_REF_CNT_PAGE, 0, &reb_ref_cnt_64, 1);

	// (4.2) Return the current (updated) reboot counter
	return reb_cnt;
}


bool_t msg_build_header(byte_t msg_type, send_msg_t* o_msg){
	o_msg->msg_buf[0] = msg_type;

	if(msg_type == PAYLOAD_SIGNUP){
			// TODO: Check if this line is correct
			o_msg->msg_buf[1] = LEN_PAYLOAD_SIGNUP;
			o_msg->msg_buf[2] = 0;

//			o_msg->len = HEADER_LEN + LEN_PAYLOAD_SIGNUP;

			return TRUE;
		}
	else if(msg_type == PAYLOAD_CHALLENGE){
		// TODO: Check if this line is correct
		o_msg->msg_buf[1] = LEN_PAYLOAD_CHALLENGE;																							// NOTE: uint16_t has 2 bytes, but our messages (so far) have lengths that fit into one byte ==> Just write to that one byte
		o_msg->msg_buf[2] = 0;

//		o_msg->len = HEADER_LEN + LEN_PAYLOAD_CHALLENGE;

		return TRUE;
	}
	else if(msg_type == PAYLOAD_GRANTED){
		// TODO: Check if this line is correct
		o_msg->msg_buf[1] = LEN_PAYLOAD_GRANTED;
		o_msg->msg_buf[2] = 0;

//		o_msg->len = HEADER_LEN + LEN_PAYLOAD_GRANTED;

		return TRUE;
	}

	return FALSE;

}


// - - - - - - - - - - - - - - -
// Message building
// - - - - - - - - - - - - - - -

// Builds a challenge message given the challenge stored in the request state
bool_t msg_build(byte_t msg_type, req_t* req_state, send_msg_t* o_msg){

	/**
	 * Recall message format:
	 * | 3 bytes header | payload |
	 */

	// Check for error in header creation ==> Sign that passed msg_type is incorrect
	if(!msg_build_header(msg_type, o_msg)){
		while(TRUE){
			__NOP();
		}
	}

	byte_t* o_payload = o_msg->msg_buf + 3;

	switch(msg_type){
	case PAYLOAD_SIGNUP:
		o_msg->len = HEADER_LEN + LEN_PAYLOAD_SIGNUP;
		memcpy(o_payload, &device_type, sizeof(device_type));

		#ifdef PROFILING_SIGNUP_FIRST_MESSAGE
			//			PROFILING_SET_PIN(gpio_profiling_pin_2);
			// 1: Prepopulated data for first signup message
			PROFILING_SET_COUNTER_F(1);
		#endif

		// NEW: Create first pairing message
		pairing_create_first_message(o_payload + 2);

		return TRUE;

	case PAYLOAD_CHALLENGE:
		o_msg->len = HEADER_LEN + LEN_PAYLOAD_CHALLENGE;
		memcpy(o_payload, req_state->challenge, LEN_PAYLOAD_CHALLENGE);
		return TRUE;

	case PAYLOAD_GRANTED:
		o_msg->len = HEADER_LEN + LEN_PAYLOAD_GRANTED;
		memcpy(o_payload, &req_state->access_type, LEN_PAYLOAD_GRANTED);
		return TRUE;

	default:
		return FALSE;
	}

}

/**
 * Reads from rx_rbuf and depending on the current state (0, 1, 2) reads header fields "msg_type", "payload_len" or the payload itself.
 * It internally resets the state to 0 after state 2 has been processed successfully.
 *
 *
 */
bool_t process_bytes(){

	bool_t success;

//	switch (state) {
//		case 0:
//			uint8_t rx_byte;
//			success = rb_consume_one(&rx_rbuf, &rx_byte);
//			if(!success){
//			return FALSE;									// return FALSE if no byte was read. NOT an error per se, could also be that nothing was received.
//			}
//
//			if(rx_byte > PAYLOAD_CONTROL){								// Received what was believed to be the payload byte, but it is not!
//				// TODO: Handle error
//			}
//			((uint8_t*)rx_msg_buf)[0] = rx_byte;
//			state ++;
//			return TRUE;
//
//		case 1:
//			success = rb_consume(&rx_rbuf, &rx_msg_payload_len, 2);		// msg_payload_len is a global variable holding the length of the currently received message
//
//			if(!success){
//				return FALSE;									// Failed reading the length, typically because both bytes have not been written yet.
//			}
//
//			if(rx_msg_payload_len > MAX_MSG_LEN) return FALSE;					// TODO: Make a better message length check
//
//	//		memcpy(((uint8_t*)rx_msg_buf) + 1, msg_payload_len, 2);		// Copy payload length into rx_msg_buf, the buffer holding the currently received message
//
//			state ++;
//			return TRUE;
//
//		case 3:
//			success = rb_consume(&rx_rbuf, ((uint8_t*) rx_msg_buf) + 3, rx_msg_payload_len);
//			if(!success){
//				return FALSE;
//			}
//			return TRUE;
//
//		default:
//			return FALSE;
//	}

	if(process_bytes_state == 0){																				// Try to receive a new message
		uint8_t rx_byte;
		success = rb_consume_one(&rx_rbuf, &rx_byte);
		if(!success){
		return FALSE;																			// return FALSE if no byte was read. NOT an error per se, could also be that nothing was received.
		}

		if(rx_byte > PAYLOAD_CONTROL){															// Received what was believed to be the payload byte, but it is not!
			// TODO: Handle error
		}
		((uint8_t*)rx_msg_buf)[0] = rx_byte;
		process_bytes_state ++;

	} else if(process_bytes_state == 1){																		// Try to receive message length
		success = rb_consume(&rx_rbuf, &rx_msg_payload_len, 2);									// msg_payload_len is a global variable holding the length of the currently received message

		if(!success){
			return FALSE;																		// Failed reading the length, typically because both bytes have not been written yet.
		}

		if(rx_msg_payload_len > MAX_PAYLOAD_LEN) return FALSE;										// TODO: Make a better message length check

//		memcpy(((uint8_t*)rx_msg_buf) + 1, msg_payload_len, 2);									// Copy payload length into rx_msg_buf, the buffer holding the currently received message

		process_bytes_state ++;

	} else if (process_bytes_state == 2){																		// Try to receive the whole message payload
		success = rb_consume(&rx_rbuf, ((uint8_t*) rx_msg_buf) + 3, rx_msg_payload_len);
		if(!success){
			return FALSE;																		// Failed ==> Have not read ANYTHING ==> Not need to e.g. revert any state!
		}

		// If we reached here, we have successfully extracted the message of length rx_msg_payload_len!
		flag_msg_rdy = TRUE;																	// Set flag to signal the new message can be extracted

		process_bytes_state = 0;																	// Reset state to be ready for the next message to arrive

		return TRUE;
	}
	else{
		return FALSE;
	}

	return TRUE;

}

// - - - - - - - - - - - - - - -
// Generic helper functions
// - - - - - - - - - - - - - - -


// Computes ceil(x/d)
uint32_t ceil_div(uint32_t x, uint32_t d){
	return 1 + ((x - 1) / d);
}

int min(int a, int b){
	return (a < b) ? a : b;
}

int16_t mod(int16_t x, int16_t m){
	int16_t y = x % m;

	return (y >= 0) ? y : (m + y);									// If y < 0, then we need to compute m - |y| which is equivalent to m + y!
}
