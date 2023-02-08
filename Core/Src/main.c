/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Set up flash
  flash_set_waitstates();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  // Wait 200 ms to prevent debugger from screwing with the reboot counter
//  HAL_Delay(200);

  // Unlock flash register FLASH_NSSR (NS for nonsecure world)
  HAL_FLASH_Unlock();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  usart2 = USART2;
  // Enable GPIO G and LPUART1 wrt. RCC. Otherwise, we would not be able to interact with the peripherals at all!
  rcc_enable_gpiod();
  rcc_enable_usart2();

  rcc_set_vddio2();

//  debug_gpio_set_g7_output();
//  while(1){
//	  GPIOG->ODR ^= 1 << 7;
//	  HAL_Delay(2);
//  }

// Initialize global objects

  device_type = 0;

  process_bytes_state = 0;

  tx_rbuf.cons_ind	= 0;
  tx_rbuf.prod_ind  = 0;
  tx_rbuf.empty		= TRUE;
  tx_rbuf.isr_ctr   = 0;
  memset(tx_rbuf.buf, 'a', sizeof(tx_rbuf.buf));

  rx_rbuf.cons_ind	= 0;
  rx_rbuf.prod_ind  = 0;
  rx_rbuf.empty		= TRUE;
  rx_rbuf.isr_ctr	= 0;
  memset(rx_rbuf.buf, 'b', sizeof(tx_rbuf.buf));

  NVIC_EnableIRQ(USART2_IRQn);

  // Initialize GPIO ports to be mapped to USART2 peripheral
  gpio_init_uart(usart2);

  // Initialize LPUART1 peripheral
  usart_init_cr1(usart2);
  usart_init_cr2(usart2);
  usart_init_cr3(usart2);
  usart_init_baudrate(usart2, USART_BAUD_RATE);


  usart_enable(usart2);

  // NOTE: MUST set TE bit (Transmit enable) BEFORE writing data to TDR! (C.f.: Page 1652, "Character Transmission" of LPUART section in reference manual)
  usart_enable_transmit(usart2);
  usart_enable_receive(usart2);


//  while(TRUE){
//	  lpuart->TDR = 'a';
//	  while(!(lpuart->ISR & USART_ISR_TXE_Msk)){
//		  __NOP();
//	  }
//  }

  usart_set_RXNEIE(usart2);

  // Set up signup button
  // Initialize signup prompt pin
  rcc_enable_gpioc();
  gpio_button_port = GPIOC;
  gpio_button_pin = GPIO_IDR_ID13;
  gpio_init_button();


  // Set up profiling
  rcc_enable_gpioe();
  gpio_profiling_port_0 = GPIOE;

  rcc_enable_gpiof();
  gpio_profiling_port_1 = GPIOF;

  gpio_init_port_as_output(gpio_profiling_port_0);
  gpio_init_port_as_output(gpio_profiling_port_1);

//  gpio_profiling_port_0->B

#ifdef PROFILING_MACRO_SIGNUP_TIME
	  label_profiling_macro_signup_time:
#endif
  gpio_profiling_pin_0.port = gpio_profiling_port_0;     // PE2 on MCU, D56/SAI_A_MCLK on board
  gpio_profiling_pin_0.pin = GPIO_PIN_2;
  PROFILING_RESET_PIN(gpio_profiling_pin_0);

  gpio_profiling_pin_1.port = gpio_profiling_port_0;     // PE4 on MCU, D57/SAI_A_FS   on board
  gpio_profiling_pin_1.pin = GPIO_PIN_4;
  PROFILING_RESET_PIN(gpio_profiling_pin_1);

  gpio_profiling_pin_2.port = gpio_profiling_port_0;     // PE5 on MCU, D58/SAI_A_SCK  on board
  gpio_profiling_pin_2.pin = GPIO_PIN_5;
  PROFILING_RESET_PIN(gpio_profiling_pin_2);

  gpio_profiling_pin_3.port = gpio_profiling_port_0;     // PE6 on MCU, D59/SAI_A_SD   on board
  gpio_profiling_pin_3.pin = GPIO_PIN_6;
  PROFILING_RESET_PIN(gpio_profiling_pin_3);

  gpio_profiling_pin_4.port = gpio_profiling_port_0;     // PE3 on MCU, D60/SAI_B_SD   on board
  gpio_profiling_pin_4.pin = GPIO_PIN_3;
  PROFILING_RESET_PIN(gpio_profiling_pin_4);

  gpio_profiling_pin_5.port = gpio_profiling_port_1;     // PF8 on MCU, D56/SAI_B_SCK on board
  gpio_profiling_pin_5.pin = GPIO_PIN_8;
  PROFILING_RESET_PIN(gpio_profiling_pin_5);

  gpio_profiling_pin_6.port = gpio_profiling_port_1;     // PF7 on MCU, D56/SAI_B_MCLK on board
  gpio_profiling_pin_6.pin = GPIO_PIN_7;
  PROFILING_RESET_PIN(gpio_profiling_pin_6);

  gpio_profiling_pin_7.port = gpio_profiling_port_1;     // PF9 on MCU, D56/SAI_B_FS on board
  gpio_profiling_pin_7.pin = GPIO_PIN_9;
  PROFILING_RESET_PIN(gpio_profiling_pin_7);


//  byte_t pin2 = (4 & 0b1) << 2;
//
//  byte_t pin4 = (4 & 0b10) << 3;
//
//  byte_t pins56 = (4 & 0b1100) << 3;
//
//  SET_BIT(gpio_profiling_port_0->BSRR, (0xFFFF<<16) | GPIO_PIN_4);
//
//  HAL_Delay(1000);

//  while(TRUE){
//	  __NOP();
//  }

//  PROFILING_SET_PIN(gpio_profiling_pin_7);
//  HAL_Delay(1000);
//  PROFILING_RESET_PIN(gpio_profiling_pin_7);
//  HAL_Delay(2000);
//
//  goto label_profiling_macro_signup_time;
//
//  gpio_profiling_pin_2 = GPIO_PIN_5;		// PE5 on MCU, D58/SAI_A_SCK  on board
//  PROFILING_RESET_PIN(gpio_profiling_pin_2);
//
//  gpio_profiling_pin_3 = GPIO_PIN_6;		// PE6 on MCU, D59/SAI_A_SD   on board
//  PROFILING_RESET_PIN(gpio_profiling_pin_3);
//
//  gpio_profiling_pin_4 = GPIO_PIN_3;		// PE3 on MCU, D60/SAI_B_SD   on board
//  PROFILING_RESET_PIN(gpio_profiling_pin_4);


  // Unused ports
  //  gpio_profiling_pin_5;
  //  gpio_profiling_pin_6;
  //  gpio_profiling_pin_7;


//  byte_t rx_byte;
  msg_t msg_state;
  req_t req_state;
  req_state.valid = FALSE;
  req_state.timeout = 10000000;													// LOOONG
  memset(req_state.challenge, sizeof(req_state.challenge), 'c');				// WARNING, this memset is ACTUALLY WRONG!! The number to set it with comes BEFORE the length!

  memset(req_state.randomness, 0, sizeof(req_state.randomness));				// Initial value of randomness is all-zero

  rng_init();

  hash_startup();

  // Key buffer holding HMAC key (note: NO LONGER padded with ZEROs to block size (512 bits == 64 bytes)
  memset(cc_struct.key_gw_s, 0x00, HMAC_KEY_LEN);
  memset(cc_struct.key_s_gw, 0xFF, HMAC_KEY_LEN);
  cc_struct.paired = FALSE;

  // Struct holding the reboot and reset counters
  cntrs_struct.reb_cnt = reb_cnt_inc_and_get();
  cntrs_struct.req_cnt = 0;

  // Message buffer holding the outbound (to the main CPU) messages
  send_msg_t o_msg;

  timer_init(TIM7, 5 * (11000 - 1));


//  // - - - - - - - - - - - - - DEBUG - - - - - - - - - - - - -
//  send_msg_t debug_msg;
//
//  flag_send_message = TRUE;
//  task_send(&debug_msg);
//
//  __NOP();
//
//  while(TRUE){
//	  __NOP();
//  }
//  // - - - - - - - - - - - - - DEBUG over - - - - - - - - - - - - -

  // SIGNUP: Send signup request (holding device type) to CPU

  // Wait until initial pairing button press

  test_label:

  #if defined(PROFILING_SIGNUP_FIRST_MESSAGE) || defined(PROFILING_SIGNUP_SECOND_MESSAGE) || defined(PROFILING_MACRO_SIGNUP_TIME) || defined(PROFILING_INFORMAL_SIGNUP)
  	  HAL_Delay(1);
  #else
  while(!button_task_check_pressed());
  #endif


#ifdef PROFILING_INFORMAL_SIGNUP
	  //			PROFILING_SET_PIN(gpio_profiling_pin_2);
	  // 1: Start signup
	  PROFILING_SET_COUNTER_F(1);
#endif

  #ifdef PROFILING_MACRO_SIGNUP_TIME
	  PROFILING_SET_PIN(gpio_profiling_pin_0);
  #endif

  // (1) Allocate message struct which will be populated
  send_msg_t signup_msg_struct;

  // (2) Build message. req_state is NULL because for signups it is not used.
  #ifdef PROFILING_SIGNUP_FIRST_MESSAGE
  //			PROFILING_SET_PIN(gpio_profiling_pin_2);
  	// Start profiling...
  	PROFILING_SET_COUNTER_F(0);
  #endif
	msg_build(PAYLOAD_SIGNUP, NULL, &signup_msg_struct);
  #ifdef PROFILING_SIGNUP_FIRST_MESSAGE
	  //			PROFILING_SET_PIN(gpio_profiling_pin_2);
	  // 6: Created signup request
	  PROFILING_SET_COUNTER_F(7);
  #endif

  // (3) Send out signup message
  flag_send_message = TRUE;
  bool_t send_success = task_send(&signup_msg_struct);
  if(! send_success){
	  // Case: We were not able to enqueue data into ring buffer
	  while(TRUE){
		  __NOP();
	  }
  }

  #if defined(PROFILING_SIGNUP_FIRST_MESSAGE)
  	  goto test_label;
  	  // Go back up to just before the pairing ==> Continuously loop through first part of pairing procedure
//  PROFILING_RESET_PIN(gpio_profiling_pin_0);
//    HAL_Delay(320);
//	NVIC_SystemReset();
  #endif

  // (4) Prepare and wait for signup response message
  byte_t signup_response_buf[HEADER_LEN + LEN_PAYLOAD_SIGNUP_RESP];


  // (4.1) Wait for signup response to be received
  while(! rb_consume(&rx_rbuf, signup_response_buf, HEADER_LEN + LEN_PAYLOAD_SIGNUP_RESP));

#ifdef PROFILING_MACRO_SIGNUP_TIME
	  PROFILING_SET_PIN(gpio_profiling_pin_4);
#endif


  #ifdef PROFILING_SIGNUP_SECOND_MESSAGE
	    // 0: Received signup response from server (manager)
	    PROFILING_SET_COUNTER_F(0);
  #endif

  // (4.2) Process signup response
  pairing_process_second_message(signup_response_buf + HEADER_LEN);

  // Paired from Gateway's point of view!
  cc_struct.paired = TRUE;

#ifdef PROFILING_MACRO_SIGNUP_TIME
	  PROFILING_SET_PIN(gpio_profiling_pin_5);
#endif

  // If we reached here, we have agreeing keys between gateway and server

  // (5) Send dummy request so server gets key confirmation
//  signup_msg_struct.len = 3 + LEN_PAYLOAD_REQUEST;
//  msg_build_header(PAYLOAD_CHALLENGE, &signup_msg_struct);
  proc_set_req_state(0x69, &req_state, &cc_struct, &cntrs_struct);
  msg_build(PAYLOAD_CHALLENGE, &req_state, &signup_msg_struct);

  flag_send_message = TRUE;
  send_success = task_send(&signup_msg_struct);
  if(! send_success){
	  // Case: We were not able to enqueue data into ring buffer
	  while(TRUE){
		  __NOP();
	  }
  }

#ifdef PROFILING_MACRO_SIGNUP_TIME
//	  HAL_Delay(50);
	  HAL_Delay(100);
	  goto label_profiling_macro_signup_time;
#endif

#ifdef PROFILING_SIGNUP_SECOND_MESSAGE
	  goto test_label;
	  // Go back up to just before the pairing ==> Continuously loop through first part of pairing procedure
//  PROFILING_RESET_PIN(gpio_profiling_pin_0);
//    HAL_Delay(320);
//	NVIC_SystemReset();
#endif


#ifdef PROFILING_INFORMAL_SIGNUP
	  // 0: Finished signup
	  PROFILING_SET_COUNTER_F(0);
#endif

#if defined(PROFILING_INFORMAL_SIGNUP)
	  goto test_label;
	  // Go back up to just before the pairing ==> Continuously loop through first part of pairing procedure
//  PROFILING_RESET_PIN(gpio_profiling_pin_0);
//    HAL_Delay(320);
//	NVIC_SystemReset();
#endif


  __NOP();

  //  byte_t signup_garbage_buf[5];
  //  while(! rb_consume(&rx_rbuf, signup_garbage_buf, 5)){
  //	  __NOP();
  //  }

  __NOP();

  // Indicator for successful message processing
  bool_t process_success = FALSE;

  while(TRUE){

	  // Check if re-pairing button pressed
	  if(button_task_check_pressed()){
		  // Create signup request
		  msg_build(PAYLOAD_SIGNUP, NULL, &signup_msg_struct);

		  // Set flag so send task actually sends something
		  flag_send_message = TRUE;

		  // Give data to send task
		  send_success = task_send(&signup_msg_struct);
		  if(! send_success){
			  // Case: We were not able to enqueue data into ring buffer
			  while(TRUE){
				  __NOP();
			  }
		  }

	  }

	  // Try to receive a message
	  task_receive(&msg_state);

	  // Receival successful <==> flag_msg_rdy flag set
	  if(!flag_msg_rdy){
		  continue;
	  }

	  // Process received message
	  process_success = task_process_msg(&msg_state, &req_state, &cc_struct, &cntrs_struct, &o_msg);


	  // TODO: Figure out if this check works. I am quite sure it should...
//	  if(!process_success){
//		continue;
//	  }

	  // Should send something to main CPU <==> flag_send_message == TRUE
	  if(!flag_send_message){
		  continue;
	  }

	  // Send something to main CPU
	  if(!task_send(&o_msg)){
		  while(TRUE){
			  __NOP();
		  }
	  }
  }

//   Sample randomness testing code
//  int i = 0;
//  while(TRUE){
//	  while(!READ_BIT(RNG->SR, RNG_SR_DRDY)){
//		  __NOP();
//	  }
//	  READ_REG(RNG->DR);
//	  i++;
//  }

  // IF WORKING CORRECTLY, THIS SHOULD BASICALLY BE AN ECHO "SERVER"
//  while(TRUE){
//	  if(! rb_consume_one(&rx_rbuf, &rx_byte)) continue;																					// If we couldn't receive anything from receive ring buffer, i.e. it is empty, we have nothing to send and can therefore just skip the remainder of this loop iteration
//
//	  if(!rb_produce_one(&tx_rbuf, rx_byte)){																								// If we received a byte but cannot enqueue it into the produce ring buffer, then the sending was too slow.
//		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	// This should NOT happen in this test setup due to the speed with which chars are read should be slow!
//		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	// ==> If we reach here, we likely have a bug in the receiver (receiving garbage/things too fast) or sender (sending too slow) or in the ring buffers (not advancing the indices correctly)
//		  __NOP();
//		  continue;																															// Don't invoke the ISR if we were not able to populate the send buffer
//	  }
//
//	  // If we reach here, we know we have successfully consumed a byte from receive buffer and enqueued it into send buffer
//	  usart_set_TXEIE(lpuart);
////	  HAL_Delay(1);
//  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
