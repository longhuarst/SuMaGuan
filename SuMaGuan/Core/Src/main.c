/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../../cylib/cylib.h"
#include "../../cylib/buffer/cylib_ringbuffer.h"
#include <string.h>

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


uint8_t uart1_rx_data = 0;
bool flag_1hz = false;
char upload_buffer[256];
ring_buffer_t uart1_rx_rb;


void delay(uint32_t t)
{
	uint8_t i = 7;
	
	while(t--){
		i=7;
		while(i--);
	}
}


bool forward_flag = true;


 void ppm1_callback()
{
	if (!forward_flag){
		return;
	}
	
	forward_flag = false;
	
	//正转
	
	
	 sprintf(upload_buffer,"\r\npub/085e2d2c-77ed-416e-b852-5f83b0392fd6/VER=1.0&TYPE=sumaguan&PAYLOAD=s/%s$\r\n","oning");
	  
		HAL_GPIO_WritePin(RE485_DE_GPIO_Port,RE485_DE_Pin,GPIO_PIN_SET);
			HAL_UART_Transmit_IT(&huart1,
			(uint8_t *)upload_buffer,
		  strlen(upload_buffer));
	
	
	
	TIM3->CNT =0 ;
	
	cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.dir,GPIO_PIN_RESET);
	while(TIM3->CNT < 7000/360.0f*100){
		cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_RESET);
		delay(10);
		cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_SET);
		delay(10);
	}
	
	
}

 void ppm2_callback()
 {
	if (forward_flag){
		return;
	}
	
	forward_flag = true;
	
	
		//反转
	
	 sprintf(upload_buffer,"\r\npub/085e2d2c-77ed-416e-b852-5f83b0392fd6/VER=1.0&TYPE=sumaguan&PAYLOAD=s/%s$\r\n","offing");
	  
		HAL_GPIO_WritePin(RE485_DE_GPIO_Port,RE485_DE_Pin,GPIO_PIN_SET);
			HAL_UART_Transmit_IT(&huart1,
			(uint8_t *)upload_buffer,
		  strlen(upload_buffer));
	
	
	TIM3->CNT =0 ;
	
	cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.dir,GPIO_PIN_SET);
	while((int16_t)TIM3->CNT > -(7000/360.0f*100)){
		cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_RESET);
		delay(10);
		cylib_step_motor_io_write(cylib_step_motor.instance[0].pin.clk,GPIO_PIN_SET);
		delay(10);
	}
	
 }
	

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  
  ring_buffer_init(&uart1_rx_rb);
  /* USER CODE BEGIN 2 */
  
  
  //HAL_Delay(30*1000);

	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
	
	HAL_UART_Receive_DMA(&huart1,&uart1_rx_data,1);
	HAL_GPIO_WritePin(RE485_DE_GPIO_Port,RE485_DE_Pin,GPIO_PIN_RESET);

  
		HAL_GPIO_WritePin(RE485_DE_GPIO_Port,RE485_DE_Pin,GPIO_PIN_SET);
	sprintf(upload_buffer,"\r\nsub/085e2d2c-77ed-416e-b852-5f83b0392fd6-device\r\n");
			HAL_UART_Transmit_IT(&huart1,
			(uint8_t *)upload_buffer,
		  strlen(upload_buffer));
	
	//while(1);


	HAL_Delay(1000);

	cylib_init();
	
	
	//0.129dg
	


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  //ppm_polling();
	  
	  
	  if (flag_1hz == true){
		flag_1hz = false;
		 
		  
		  if (forward_flag== true){
			sprintf(upload_buffer,"\r\npub/085e2d2c-77ed-416e-b852-5f83b0392fd6/VER=1.0&TYPE=sumaguan&PAYLOAD=s/%s$\r\n","off");
		  }else{
			sprintf(upload_buffer,"\r\npub/085e2d2c-77ed-416e-b852-5f83b0392fd6/VER=1.0&TYPE=sumaguan&PAYLOAD=s/%s$\r\n","on");
		  }
		  
		HAL_GPIO_WritePin(RE485_DE_GPIO_Port,RE485_DE_Pin,GPIO_PIN_SET);
			HAL_UART_Transmit_IT(&huart1,
			(uint8_t *)upload_buffer,
		  strlen(upload_buffer));
		  
		  
		  
		  
	  }
	  
	  
	  
	  
	  //解析串口数据
	  {
		static char buffer[128];
		 static uint8_t pos = 0;
		
		  pos += ring_buffer_dequeue(&uart1_rx_rb,&buffer[pos]);
		  
		  if (pos >= 1){
			if (buffer[pos-1] == '\n'){
				
				
				buffer[pos] = 0;
				
				
				if (strcmp(buffer,"VER=1.0&TYPE=sumaguan&PAYLOAD=c/on$\r\n") ==0){
					ppm1_callback();
				}else if (strcmp(buffer,"VER=1.0&TYPE=sumaguan&PAYLOAD=c/off$\r\n") ==0){
					ppm2_callback();
				}
				
				
				
				pos = 0;
			
			}
		  }
		  
		  
		  if (pos >= 128){
			pos = 0;
		  }
		  
		  
		  
		  
	  }
	  
	  
	  
	  
	  
	  
	  
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	
	if(htim->Instance == TIM2){
		ppm_interrupt();
	}else if(htim->Instance == TIM4){
		static uint32_t counter = 0;
		
		counter++;
		counter%=1000;
		
		if (counter == 0){
			flag_1hz = true;
		}
		
		
		
	}

}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1){
		HAL_GPIO_WritePin(RE485_DE_GPIO_Port,RE485_DE_Pin,GPIO_PIN_RESET);
	}
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART1){
		ring_buffer_queue(&uart1_rx_rb,uart1_rx_data);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
