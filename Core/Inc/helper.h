/*
 * helper.h
 *
 *  Created on: Mar 28, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_HELPER_H_
#define INC_HELPER_H_

/* USER CODE BEGIN EFP */
#include "main.h"

void rcc_enable_gpiog();
void rcc_enable_gpiod();
void rcc_enable_gpioc();
void rcc_enable_gpioe();
void rcc_enable_gpiof();
void rcc_enable_lpuart();
void rcc_enable_usart2();
void rcc_set_vddio2();

void gpio_init_uart(USART_TypeDef * uart);
void gpio_init_button();
void gpio_init_port_as_output(GPIO_TypeDef * gpio);

void usart_init_cr1(USART_TypeDef * uart);
void usart_init_cr2(USART_TypeDef * uart);
void usart_init_cr3(USART_TypeDef * uart);
void usart_init_baudrate(USART_TypeDef * uart, uint32_t baudrate);

void usart_enable(USART_TypeDef * uart);
void usart_enable_receive(USART_TypeDef * uart);
void usart_enable_transmit(USART_TypeDef * uart);
void usart_set_TXEIE(USART_TypeDef * uart);
void usart_set_RXNEIE(USART_TypeDef * uart);

void   rb_init(ringbuf_t * rb, char init_val);
bool_t rb_consume_one(ringbuf_t* rb, byte_t* obyte);
bool_t rb_produce_one(ringbuf_t* rb, byte_t ibyte);

int16_t rb_consume_all(ringbuf_t* rb, byte_t* obuf);

bool_t rb_consume(ringbuf_t* rb, byte_t * obuf, int16_t num_items);
bool_t rb_produce(ringbuf_t* rb, byte_t* ibuf, int16_t num_items);

bool_t rx_check_type(byte_t b);
bool_t rx_check_len(uint16_t len, byte_t b);

void rng_init();
void rng_get_randomness(byte_t* o_buf, uint32_t num_samples);

void hash_startup();
void hash_hmac(byte_t* key_buf, byte_t* msg_buf, uint32_t len_key, uint32_t len_msg, byte_t* o_buf);
void hash_hkdf(
		byte_t* ikm,
		byte_t* salt,
		byte_t** infos,
		uint32_t ikm_len,
		uint32_t salt_len,
		uint32_t info_len,
		uint32_t infos_num,
		byte_t** o_bufs);

void timer_init(TIM_TypeDef* t, uint16_t presc);
void timer_start(TIM_TypeDef* t, uint16_t timeout);

void flash_set_waitstates();
void flash_erase_page_b1(uint8_t flash_target_page);
void flash_program_page_b1(uint8_t flas_target_page, uint16_t byte_offset, uint64_t* dword_input, uint32_t num_dwords);
void flash_read_b1(uint8_t flash_target_page, uint16_t byte_offset, uint64_t* o_buf, uint32_t num_dwords);
uint64_t* flash_get_addr(uint8_t flash_target_page, uint16_t byte_offset);
uint64_t* flash_get_page_addr(uint8_t flash_target_page);

uint32_t reb_cnt_inc_and_get();
bool_t msg_build(byte_t msg_type, req_t* req_state, send_msg_t* o_msg);
bool_t process_bytes();

// Generic helper functions
int min(int a, int b);
int16_t mod(int16_t x, int16_t m);

#endif /* INC_HELPER_H_ */
