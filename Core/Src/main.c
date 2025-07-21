/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DAQ.h"
#include "Position.h"
#include "Proximity.h"
#include "IMU.h"
#include "GPS.h"
#include "Temperature.h"
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
daq_timestamp_t g_timestamp;
extern DMA_HandleTypeDef hdma_tim3_ch1_trig, hdma_tim3_ch2, hdma_tim3_ch3, hdma_tim3_ch4_up;
bool g_i2c_dma_flags[DAQ_NO_OF_I2C_DMA_DEVICES];
daq_i2c_dma_devices_t g_i2c_dma_device = I2C_DMA_NO_DEVICE;
BaseType_t task_returns[DAQ_NO_OF_TASKS];
TaskHandle_t* task_handles[DAQ_NO_OF_TASKS];
SemaphoreHandle_t g_i2c_mutex;
CAN_TxHeaderTypeDef can_tx_header;
DMA_HandleTypeDef *proximity_dma_handlers[4] = {&hdma_tim3_ch1_trig, &hdma_tim3_ch2, &hdma_tim3_ch3, &hdma_tim3_ch4_up};

imu_axis_map_t imu_axis_map = {
  .x = BNO055_AXIS_X,
  .x_sign = BNO055_AXIS_SIGN_POSITIVE,
  .y = BNO055_AXIS_Y,
  .y_sign = BNO055_AXIS_SIGN_POSITIVE,
  .z = BNO055_AXIS_Z,
  .z_sign = BNO055_AXIS_SIGN_POSITIVE
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Task1Blink(void *pvParameters);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  g_i2c_mutex  = xSemaphoreCreateMutex();
  if (g_i2c_mutex == NULL)
	  Error_Handler();
  ADC_Sensors_Init(&hadc1);
  Prox_Init(&htim3, proximity_dma_handlers);
  IMU_Init(&hi2c1, OPERATION_MODE_NDOF, imu_axis_map);
  GPS_Init(&hi2c1);
  Temp_Init(&hi2c1);
  HAL_CAN_Start(&hcan1);
  DAQ_CAN_Init(&hcan1, &can_tx_header);
  xTaskCreate(Task1Blink, "Blink1", 64, NULL, 5, NULL);
  task_returns[ADC_TASK] = xTaskCreate(ADC_Task		, "ADC_Task" , 2*256 , NULL, 3, task_handles[ADC_TASK]);
  task_returns[PROX_TASK]= xTaskCreate(Prox_Task	, "Prox_Task", 256	 , NULL, 1, task_handles[PROX_TASK]);
  task_returns[IMU_TASK] = xTaskCreate(IMU_Task		, "IMU_Task" , 2*256 , NULL, 2, task_handles[IMU_TASK]);
  task_returns[GPS_TASK] = xTaskCreate(GPS_Task		, "GPS_task" , 256	 , NULL, 3, task_handles[GPS_TASK]);
  task_returns[TEMP_TASK]= xTaskCreate(Temp_Task 	, "Temp_task", 256 	 , NULL, 4, task_handles[TEMP_TASK]);
  task_returns[CAN_TASK] = xTaskCreate(DAQ_CAN_Task	, "CAN_task" , 256	 , NULL, 5, task_handles[CAN_TASK]);
  vTaskStartScheduler();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Task1Blink(void *pvParameters)
{
    for(;;)
    {
    	vTaskDelay(20);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(!(g_i2c_dma_flags[g_i2c_dma_device]) && g_i2c_dma_device != I2C_DMA_NO_DEVICE)
	{
		g_i2c_dma_flags[g_i2c_dma_device] = true;
		g_i2c_dma_device = I2C_DMA_NO_DEVICE;
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM7)
  {
	  g_timestamp.counter++;
	  if(g_timestamp.counter==1000)
	  {
		  g_timestamp.counter=0;
	      g_timestamp.seconds++;
	      if(g_timestamp.seconds==60)
	      {
	    	  g_timestamp.seconds = 0;
	    	  g_timestamp.minutes++;
	    	  if(g_timestamp.minutes==60)
	    	  {
	    		  g_timestamp.minutes=0;
	    		  g_timestamp.hours++;
	    		  if(g_timestamp.hours==24)
	    			  g_timestamp.hours=0;
	    	  }
	      }
	  }
  }
  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
