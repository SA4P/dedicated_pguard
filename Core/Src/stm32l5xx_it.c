/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l5xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l5xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

// TIM1 Update Event (UP) IRQ handler

//int x = 0;

void TIM1_UP_IRQHandler(){
//	SET_BIT(DBGMCU->APB2FZR, DBGMCU_APB2FZR_DBG_TIM1_STOP);
	__NOP();
}

void TIM7_IRQHandler(){
	// Set currently granting flag to false
	flag_currently_granting = FALSE;

	// Clear update event interrupt flag
	CLEAR_BIT(TIM7->SR, TIM_SR_UIF);

	// Disable interrupt generation for update events. This is NECESSARY because within timer_start we actually trigger an update event to commit values into the shadow registers.
	// If we had this bit set, then even if the global interrupt is disabled, an interrupt for this peripheral would be "enqueued" and would fire upon NVIC_EnableIRQ(TIM7_IRQn)!
	CLEAR_BIT(TIM7->DIER, TIM_DIER_UIE);

	// Because we are NOT using one-pulse mode anymore, we now have to explicitly disable the timer!
	CLEAR_BIT(TIM7->CR1, TIM_CR1_CEN);

	// Finally, disable the TIM7 update interrupt in the CPU.
	NVIC_DisableIRQ(TIM7_IRQn);

}

static int ctr = 0;

// USART2 IRQ handler
void USART2_IRQHandler(){
//	tx_rbuf.

	if(USART2->CR1 & USART_CR1_TXEIE_Msk && USART2->ISR & USART_ISR_TXE_Msk){			// Case: TD empty interrupt enabled AND TD empty flag is set


		// Case 1: Have data to send ==> Send Populate TDR with one byte
		byte_t next_byte;
		if(!rb_consume_one(&tx_rbuf, &next_byte)){
			USART2->CR1 ^= USART_CR1_TXEIE;											// Couldn't receive anything from tx_rbuf ==> Buffer empty ==> Have consumed everything we could have consumed ==> Disable interrupt until buffer has been filled with some data
			#ifdef PROFILING_MACRO_SIGNUP_TIME
				if(ctr++ == 0){
					PROFILING_SET_PIN(gpio_profiling_pin_3);
				}else{
					PROFILING_SET_PIN(gpio_profiling_pin_7);
					ctr = 0;
				}
			#endif

			#ifdef PROFILING_MACRO_AUTH_TIME
//				if(ctr < 2){
//					ctr++;
//				}else{
//					if(ctr % 2 == 0){
//						PROFILING_SET_COUNTER_F(2);
//					}else{
//						PROFILING_SET_COUNTER_F(4);
//					}
//					ctr++;
//				}
			#endif

			return;
		}
		USART2->TDR = next_byte;

		return;
	}

	if(USART2->CR1 & USART_CR1_RXNEIE_Msk && USART2->ISR & USART_ISR_RXNE_Msk){		// Case: RD not empty interrupt enabled AND RD not empty flag is set
		uint8_t rx_byte = USART2->RDR & 0xFF;											// 8 bit data ==> Mask RDR (Receive Data Register) with 0xFF.
		if(!rb_produce_one(&rx_rbuf, rx_byte)){
			// Could not put into buffer because it was full ==> Assuming there is no other bug, this means that the buffer was not cleared fast enough!
			while(1){
				__NOP();
			}
		}
	}



}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
