/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "inttypes.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define PCLK1_FREQ			((uint64_t)110000000)														// Run AHB at 110 MHz ==> PCLK1 = PCLK2 := 110MHz
#define RBUF_LEN 			512

// Needed for RNG, taken from stm32l5xx_hal_rng.h
#define RNG_HTCFG_1   0x17590ABCU /*!< Magic number */
#define RNG_HTCFG     0x0000A2B3U /*!< Recommended value for NIST compliancy */

// Profiling macro
#define PROFILING
#define PROFILING_SET_PIN(PIN)			SET_BIT((PIN.port)->BSRR, PIN.pin)
#define PROFILING_RESET_PIN(PIN)		SET_BIT((PIN.port)->BSRR, PIN.pin << 16)					// BSRR is split into 2 16 bit registers, lower 16 bits are to set a pin, upper 16 to reset

#define PROFILING_SET_COUNTER_E(AMNT)		SET_BIT(GPIOE->BSRR, 0xFFFF << 16 | ((AMNT & 0b1) << 2) | ((AMNT & 0b10) << 3) | ((AMNT & 0b1100) << 3))
#define PROFILING_SET_COUNTER_F(AMNT)		SET_BIT(GPIOF->BSRR, 0xFFFF << 16 | ((AMNT & 0b1) << 8) | ((AMNT & 0b10) << 6) | ((AMNT & 0b100) << 7))

//#define PROFILING_SIGNUP_FIRST_MESSAGE
//#define PROFILING_SIGNUP_SECOND_MESSAGE
//#define PROFILING_REQUEST
//#define PROFILING_RESPONSE
//#define PROFILING_MACRO_SIGNUP_TIME
//#define PROFILING_MACRO_AUTH_TIME
#define PROFILING_INFORMAL_SIGNUP


// - - - - - - - - - - - CONSTANTS - - - - - - - - - - -
#define TRUE 0x1
#define FALSE 0x0

#define  USART_BAUD_RATE		921600

#define MAX_PRINT_LEN			1024					// Max. length a debug message can have, This is used in: strnlen(..., MAX_PRINT_LEN)

#define SHA256_INPUT_SIZE 		64						// 512 bits
#define SHA256_OUTPUT_SIZE 		32						// 256 bits

#define HMAC_INPUT_SIZE         SHA256_INPUT_SIZE
#define HMAC_KEY_LEN 			32						// Key is recommended to have the same length as one input block, but we establish the key using ECDHE over Curve25591, yielding a 32 byte key
#define HMAC_OUTPUT_SIZE		32

#define KEY_LEN					HMAC_KEY_LEN

#define MAX_PAYLOAD_LEN			256						// Longest message we expect to receive. DANGEROUS because changes to protocol might require changes to this value!

#define REB_CNT_LEN				4
#define FLASH_REB_CNT_PAGE		(FLASH_PAGE_NB_PER_BANK - 2)	// Position of reboot counter in flash memory
#define FLASH_REB_REF_CNT_PAGE	(FLASH_REB_CNT_PAGE + 1)		// Position of reboot counter reference value (for the 2-step increment process) in flash memory

#define REQ_CNT_LEN				4
#define RANDOM_LEN				16

#define ACCESS_TYPE_LEN			2
#define CHALLENGE_LEN			(REB_CNT_LEN + REQ_CNT_LEN + ACCESS_TYPE_LEN + HMAC_OUTPUT_SIZE)	// XXX: Challenge consists of: |  REB_CNT  |  REQ_CNT  |  ACCESS_TYPE  |  MAC_TAG  |


#define HEADER_LEN				3
#define HEADER_TYPE_LEN			1
#define HEADER_LEN_LEN			2

//#ifdef PROFILING
//	#define TIMEOUT_10				0
//#else
	#define TIMEOUT_10				10						// 10ms timeout
//#endif

#define TIMER_SET_UDIS(TIMER_PTR)		SET_BIT(TIMER_PTR->CR1, TIM_CR1_UDIS)
#define TIMER_CLEAR_UDIS(TIMER_PTR)		CLEAR_BIT(TIMER_PTR->CR1, TIM_CR1_UDIS)
#define TIMER_RESET_CNT(TIMER_PTR)		CLEAR_REG(TIMER_PTR->CNT)
// - - - - - - - - - - - - - - - - - - - - - -
// 			Typedefs
// - - - - - - - - - - - - - - - - - - - - - -

typedef uint8_t byte_t;
typedef uint8_t bool_t;
typedef int8_t	err_t;
typedef uint32_t time_tt;

typedef enum{
	PAYLOAD_SIGNUP    	=	0,
	PAYLOAD_REQUEST   	=	1,
	PAYLOAD_CHALLENGE 	=   2,
	PAYLOAD_RESPONSE  	=	3,
	PAYLOAD_GRANTED	  	=	4,
	PAYLOAD_CONTROL   	=	5,
	PAYLOAD_SIGNUP_RESP =	6										// Added this for the server's handshake response message
} payload_types_t;

typedef enum{
	LEN_PAYLOAD_SIGNUP		=		(2 + KEY_LEN +KEY_LEN + HMAC_OUTPUT_SIZE),				// 32 byte static public key (already part of scan, included here as well to allow server to match scan and signup request),
																							// 32 byte ephemeral public key, 32 byte MAC tag
	LEN_PAYLOAD_REQUEST		=		2,
	LEN_PAYLOAD_CHALLENGE	=		CHALLENGE_LEN,
	LEN_PAYLOAD_RESPONSE	=		(RANDOM_LEN + HMAC_OUTPUT_SIZE),				// |  s_rand (RANDOM_LEN bytes) |  mac_tag (HMAC_OUTPUT_SIZE) |
	LEN_PAYLOAD_GRANTED		=		LEN_PAYLOAD_REQUEST,							// Notify the CPU about an access being granted
	LEN_PAYLOAD_CONTROL		=		3,												// FIXME: Set correct LEN_CONTROL
	LEN_PAYLOAD_SIGNUP_RESP	=		(KEY_LEN + HMAC_OUTPUT_SIZE)					// 32 byte public ephemeral key, 32 byte MAC tag

} payload_lens_t;

typedef enum{											// TODO: Make this structure somehow dynamic, i.e. have a mapping which is populated at signup-time of a new IoT-device
	SAMPLE_SENSOR_0 	=	0,
	SAMPLE_SENSOR_1 	=	1,
	CONTROL_ACTUATOR_0	=	2,
	CONTROL_ACTUATOR_1	=	3,
	DUMMY_REQUEST		=   0x69
} access_type_t;



// - - - - - - - - - - - - - - - - - - - - - -
// 			Structs
// - - - - - - - - - - - - - - - - - - - - - -

typedef struct ringbuf{
	volatile int16_t prod_ind;
	volatile int16_t cons_ind;
	volatile int16_t isr_ctr;																			// Ctr. shared between ISR and normal world, it tells the ISR how many more bytes it has to send before disabling itself (to prevent flood of interrupts)
	volatile bool_t   empty;
			 byte_t buf[RBUF_LEN];
}ringbuf_t;

typedef struct message {
	byte_t payload_type;
	uint32_t payload_len;
	byte_t p_payload[MAX_PAYLOAD_LEN];																	// 128 byte buffer, UNALIGNED
	time_tt rx_time;
}msg_t;


typedef struct req {
	bool_t    	valid;
	uint16_t  	access_type;
	time_tt   	timer_set_time;
	time_tt   	timeout;
	uint32_t	ref_reb_cnt;							// Reboot counter value at time when the request was generated
	uint32_t	ref_req_cnt;							// Request counter value at time when the request was generated
	byte_t*		tag_ptr;								// Challenge has format: |  reb_cnt  |  req_cnt  |  access_type  |  mac_tag  |. tag_ptr points to the beginning of the mac_tag. This is purely for convenience
	byte_t		challenge[CHALLENGE_LEN];
	byte_t		randomness[RANDOM_LEN];
} req_t;

// Struct holding message and its length to be sent
typedef struct send_message{
	byte_t 		msg_buf[MAX_PAYLOAD_LEN];
	uint32_t	len;
}send_msg_t;

typedef struct cnts{
	uint32_t	reb_cnt;
	uint32_t	req_cnt;
}cntrs_t;

typedef struct crypto_context{
	bool_t		paired;
	byte_t		shared_secret[KEY_LEN];
	byte_t		key_gw_s[KEY_LEN];
	byte_t		key_s_gw[KEY_LEN];
}crypto_context_t;

typedef struct profiling_pin{
	GPIO_TypeDef* port;
	uint32_t pin;
}profiling_pin_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// - - - - - - - - - - - - - - - - - - - - - -
// 			Global variables
// - - - - - - - - - - - - - - - - - - - - - -



USART_TypeDef* usart2;

// DEVICE TYPE
uint16_t device_type;

// FLAGS
bool_t flag_msg_rdy;
bool_t flag_send_message;
volatile bool_t flag_currently_granting;
volatile uint16_t flag_type_currently_granting;											// If flag_currently_granting == TRUE, this "flag" (not really a flag) holds the access type that is currently being granted!

int16_t process_bytes_state;
uint32_t rx_msg_buf[MAX_PAYLOAD_LEN / 4];
uint16_t rx_msg_payload_len;

// Ringbuffers for low-level send/receive routines
ringbuf_t tx_rbuf;
ringbuf_t rx_rbuf;

// Signup button
GPIO_TypeDef* gpio_button_port;
uint32_t	  gpio_button_pin;

// Variables for high-level state keeping and message processing
crypto_context_t cc_struct;
cntrs_t cntrs_struct;

// Profiling data
	GPIO_TypeDef* gpio_profiling_port_0;
	GPIO_TypeDef* gpio_profiling_port_1;

	profiling_pin_t	  gpio_profiling_pin_0;
	profiling_pin_t	  gpio_profiling_pin_1;
	profiling_pin_t	  gpio_profiling_pin_2;
	profiling_pin_t	  gpio_profiling_pin_3;
	profiling_pin_t	  gpio_profiling_pin_4;
	profiling_pin_t	  gpio_profiling_pin_5;
	profiling_pin_t	  gpio_profiling_pin_6;
	profiling_pin_t	  gpio_profiling_pin_7;


#include "helper.h"
#include "recv_task.h"
#include "proc_task.h"
#include "send_task.h"
#include "pairing_task.h"
#include "button_task.h"
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//#define UART_UE_Enable
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
