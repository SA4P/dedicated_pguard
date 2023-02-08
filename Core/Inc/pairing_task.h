/*
 * pairing_task.h
 *
 *  Created on: Sep 12, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_PAIRING_TASK_H_
#define INC_PAIRING_TASK_H_

#include "main.h"

void pairing_create_first_message(byte_t* o_buf);
void pairing_process_second_message(byte_t* i_buf);

#endif /* INC_PAIRING_TASK_H_ */
