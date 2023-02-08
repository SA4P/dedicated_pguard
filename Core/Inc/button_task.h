/*
 * button_task.h
 *
 *  Created on: Jun 13, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_BUTTON_TASK_H_
#define INC_BUTTON_TASK_H_

#include "main.h"

bool_t button_task_check_pressed();
bool_t signup_task_check_requested(req_t* req_state, send_msg_t* o_msg);

#endif /* INC_BUTTON_TASK_H_ */
