/*
 * send_task.c
 *
 *  Created on: Apr 4, 2022
 *      Author: SA4P Authors
 */

#include "send_task.h"

static int ctr = 0;
bool_t task_send(send_msg_t* o_msg){

	// Flag already checked in main loop, just double check. If it is for whatever reason not set, we have an error because we should only enter task_send if flag_send_message == TRUE!
	if(!flag_send_message){
		while(TRUE){
			__NOP();
		}
	}

	// Set flag to false
	flag_send_message = FALSE;
#ifdef PROFILING_MACRO_SIGNUP_TIME
		PROFILING_SET_PIN(gpio_profiling_pin_1);
#endif

	if(!rb_produce(&tx_rbuf, o_msg->msg_buf, o_msg->len)){
		return FALSE;
	}

#ifdef PROFILING_MACRO_SIGNUP_TIME
	if(ctr++ == 0) {
	  PROFILING_SET_PIN(gpio_profiling_pin_2);
	}else{
		PROFILING_SET_PIN(gpio_profiling_pin_6);
		ctr = 0;
	}

#endif
	// Set UART to send out message
	usart_set_TXEIE(usart2);

	return TRUE;


}
