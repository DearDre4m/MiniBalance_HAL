/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "velocity.h"
#include "qmi8658.h"
#include <stdio.h>
#include <math.h>
#include "attitude_ekf.h"
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
volatile int16_t left_encoder_count = 0;
volatile int16_t right_encoder_count = 0;

volatile uint8_t velocity_sample_ready = 0;
AttitudeEKF_t g_attitude_ekf;

volatile uint8_t imu_5ms_ready = 0;

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
  uint8_t msg[] = "MiniBalance HAL Start!\r\n";
  HAL_UART_Transmit(&huart1,msg,sizeof(msg)-1,100);




  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  Velocity_Reset();
  HAL_TIM_Base_Start_IT(&htim5);
  QMI8658_Init();
  QMI8658_Calibrate();
  HAL_Delay(20);

  float initial_bias_rad_s = QMI8658_GyroRawToRadS(gyro_x_offset);
  AttitudeEKF_Init(&g_attitude_ekf, angle_zero, initial_bias_rad_s);

  char msg1[100];
  int len = snprintf(msg1,sizeof(msg1),"GX_OFFSET:%.2f GZ_OFFSET:%.2f ANGLE_ZERO:%.2f\r\n",gyro_x_offset,gyro_z_offset,angle_zero);
  HAL_UART_Transmit(&huart1,(uint8_t *)msg1,len,100);

  uint16_t print_divider = 0;

  QMI8658_RawData_t imu;

  float ay_mps2;
  float az_mps2;
  float gyro_x_rad_s;

  float ekf_angle_deg;
  float ekf_bias_dps;

  float acc_angle_deg;

  char print_msg[200];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
  {
    if (imu_5ms_ready)
    {
      imu_5ms_ready = 0;
      /* ============================
       * 读取 IMU
       * ============================
       */
      QMI8658_ReadRaw(&imu);
      /* ============================
       * 转换成 SI 单位
       * ============================
       */
      ay_mps2 = QMI8658_AccRawToMps2(imu.acc_y);

      az_mps2 = QMI8658_AccRawToMps2(imu.acc_z);

      gyro_x_rad_s = QMI8658_GyroRawToRadS(imu.gyro_x);

      /* ============================
       * EKF
       *
       * 第一阶段：
       * 暂时 a = 0
       * ============================
       */

      AttitudeEKF_Update(&g_attitude_ekf, gyro_x_rad_s, ay_mps2, az_mps2, g_velocity.linear_acc_m_s2, 0.005f);
      /* 每 200ms 打印一次 */
      print_divider++;

      if(print_divider >= 40)
      {
        print_divider = 0;
        ekf_angle_deg = AttitudeEKF_GetAngleDeg(&g_attitude_ekf);
        ekf_bias_dps = AttitudeEKF_GetBiasDps(&g_attitude_ekf);
        /*
        * 纯加速度计角度
        * 用来和 EKF 做对照
        */
        acc_angle_deg = QMI8658_GetAccAngle(&imu);
        //int len = snprintf(print_msg,sizeof(print_msg),"ACC:%7.2f  EKF:%7.2f  BIAS:%7.3f\r\n", acc_angle_deg,ekf_angle_deg,ekf_bias_dps);
        int len = snprintf(print_msg,sizeof(print_msg),"V:%7.3f  A_RAW:%7.3f  A_F:%7.3f  ACC:%7.2f  EKF:%7.2f\r\n",g_velocity.average_m_s,g_velocity.linear_acc_raw_m_s2,g_velocity.linear_acc_m_s2,acc_angle_deg,ekf_angle_deg);
        HAL_UART_Transmit(&huart1,(uint8_t *)print_msg,len,100);
        }
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t velocity_divider = 0;

  if(htim->Instance == TIM5)
  {
    /*
     * 每 5ms 请求一次 IMU / EKF 更新
     */
    imu_5ms_ready = 1;    
    /*
    * 每 20ms 更新一次编码器速度
    */

    velocity_divider ++;
    /* 5 ms × 4 = 20 ms，也就是 50 Hz */
    if(velocity_divider >=4 )
    {
      velocity_divider = 0;

      int16_t left_count = Encoder_LeftRead();
      int16_t right_count = Encoder_RightRead();

      Velocity_Update(left_count,right_count);

      velocity_sample_ready = 1;
    }

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
