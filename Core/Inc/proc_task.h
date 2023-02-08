/*
 * proc_task.h
 *
 *  Created on: Apr 3, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_PROC_TASK_H_
#define INC_PROC_TASK_H_

#include "main.h"

bool_t task_process_msg(msg_t* msg_state, req_t* req_state, crypto_context_t* keys_struct, cntrs_t* cntrs, send_msg_t* o_msg);
bool_t	 proc_set_req_state(uint16_t access_type, req_t *req_state, crypto_context_t* keys_struct, cntrs_t* cntrs_struct);

#endif /* INC_PROC_TASK_H_ */
