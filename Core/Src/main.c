/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body – Raw sensor data + MotionFX
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
#include "main.h"
#include "app_mems.h"
#include "motion_fx.h"
#include "iks4a1_motion_sensors.h"

#include "lis2mdl.h"
#include "lsm6dso16is_reg.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MFX_STATE_SIZE 2450

extern I2C_HandleTypeDef hi2c1;
CRC_HandleTypeDef hcrc;
UART_HandleTypeDef huart3;

MFX_input_t mfx_input;
MFX_output_t mfx_output;
MFX_MagCal_input_t magCalInput;
MFX_MagCal_output_t magCalOutput;

// yaw initialization
float initial_yaw = 0.0f;
uint8_t yaw_reference_set = 0;
float relative_yaw = 0.0f;

uint32_t last_print_tick = 0;
const uint32_t PRINT_INTERVAL = 100;
uint32_t last_tick = 0;

static uint8_t mfxstate[MFX_STATE_SIZE];

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);

void convert_acc(int16_t *rawData, float *convertedData);
void convert_gyro(int16_t *rawData, float *convertedData);
void convert_mag(int16_t *rawData, float *convertedData);
void start_mag_cal(void);

void read_insert_sensors(){
    IKS4A1_MOTION_SENSOR_Axes_t raw_acc, raw_gyro, raw_mag;

	if(IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSO16IS_0, MOTION_ACCELERO, &raw_acc) == BSP_ERROR_NONE)
	{
		// MFX_input_t structure needs acc [g] gyro [dps] and [uT/50]
		mfx_input.acc[0] = raw_acc.x / 1000.0f;
		mfx_input.acc[1] = raw_acc.y / 1000.0f;
		mfx_input.acc[2] = raw_acc.z / 1000.0f;
	}

	if(IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSO16IS_0, MOTION_GYRO, &raw_gyro) == BSP_ERROR_NONE)
	{
		mfx_input.gyro[0] = raw_gyro.x / 1000.0f;
		mfx_input.gyro[1] = raw_gyro.y / 1000.0f;
		mfx_input.gyro[2] = raw_gyro.z / 1000.0f;
	}

	if(IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, &raw_mag) == BSP_ERROR_NONE)
	{
		mfx_input.mag[0] = (raw_mag.x * 0.1f / 50.0f) - magCalOutput.hi_bias[0];
		mfx_input.mag[1] = (raw_mag.y * 0.1f / 50.0f) - magCalOutput.hi_bias[1];
		mfx_input.mag[2] = (raw_mag.z * 0.1f / 50.0f) - magCalOutput.hi_bias[2];
	}
}


int32_t set_sensors_scale(){
	int32_t ret = BSP_ERROR_NONE;

	//Accelerometer

	if (IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LSM6DSO16IS_0, MOTION_ACCELERO, 2) != BSP_ERROR_NONE)
	{
		ret = BSP_ERROR_COMPONENT_FAILURE;
	}

	if (IKS4A1_MOTION_SENSOR_SetOutputDataRate(IKS4A1_LSM6DSO16IS_0, MOTION_ACCELERO, 104.0f) != BSP_ERROR_NONE)
	{
		ret = BSP_ERROR_COMPONENT_FAILURE;
	}

	//Gyroscope

	if (IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LSM6DSO16IS_0, MOTION_GYRO, 500) != BSP_ERROR_NONE)
	{
		ret = BSP_ERROR_COMPONENT_FAILURE;
	}

	if (IKS4A1_MOTION_SENSOR_SetOutputDataRate(IKS4A1_LSM6DSO16IS_0, MOTION_GYRO, 104.0f) != BSP_ERROR_NONE)
	{
		ret = BSP_ERROR_COMPONENT_FAILURE;
	}


	//Magnetometer

	if (IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, 50) != BSP_ERROR_NONE)
		{
		    ret = BSP_ERROR_COMPONENT_FAILURE;
		}

	if (IKS4A1_MOTION_SENSOR_SetOutputDataRate(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, 50.0f) != BSP_ERROR_NONE)
	{
		ret = BSP_ERROR_COMPONENT_FAILURE;
	}

	return ret;
}

void sensors_init(){
	IKS4A1_MOTION_SENSOR_Init(IKS4A1_LSM6DSO16IS_0, MOTION_ACCELERO | MOTION_GYRO);
	IKS4A1_MOTION_SENSOR_Init(IKS4A1_LIS2MDL_0, MOTION_MAGNETO);
    IKS4A1_MOTION_SENSOR_Enable(IKS4A1_LSM6DSO16IS_0, MOTION_ACCELERO);
	IKS4A1_MOTION_SENSOR_Enable(IKS4A1_LSM6DSO16IS_0, MOTION_GYRO);
	IKS4A1_MOTION_SENSOR_Enable(IKS4A1_LIS2MDL_0, MOTION_MAGNETO);

	if (set_sensors_scale() != BSP_ERROR_NONE)
	{
	    Error_Handler();
	}
}

void start_mag_cal(void)
{
    IKS4A1_MOTION_SENSOR_Axes_t mag_data;
    char msg_mag[128];
    uint32_t timeout = 0;
    const uint32_t max_attempts = 10000;
    uint8_t last_quality = 0xFF;

    HAL_UART_Transmit(&huart3, (uint8_t*)"Calibrating magnetometer...\r\n", 29, HAL_MAX_DELAY);

    do {
        IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, &mag_data);

        magCalInput.mag[0] = mag_data.x * 0.1f / 50.0f;
        magCalInput.mag[1] = mag_data.y * 0.1f / 50.0f;
        magCalInput.mag[2] = mag_data.z * 0.1f / 50.0f;
        magCalInput.time_stamp = HAL_GetTick();

        MotionFX_MagCal_run(&magCalInput);
        MotionFX_MagCal_getParams(&magCalOutput);

        if (magCalOutput.cal_quality != last_quality) {
            last_quality = magCalOutput.cal_quality;

            sprintf(msg_mag,
                "Calibration Quality: %d. Offsets (uT): X=%.2f Y=%.2f Z=%.2f\r\n",
                last_quality,
                magCalOutput.hi_bias[0] * 50.0f,
                magCalOutput.hi_bias[1] * 50.0f,
                magCalOutput.hi_bias[2] * 50.0f);

            HAL_UART_Transmit(&huart3, (uint8_t*)msg_mag, strlen(msg_mag), HAL_MAX_DELAY);
        }

        HAL_Delay(10);
        timeout++;

    } while (magCalOutput.cal_quality != MFX_MAGCALGOOD && timeout < max_attempts);

    sprintf(msg_mag,
        (magCalOutput.cal_quality == MFX_MAGCALGOOD) ?
        "Calibration SUCCESSFUL. Quality: %d\r\n" :
        "Calibration FAILED. Quality: %d\r\n",
        magCalOutput.cal_quality);

    HAL_UART_Transmit(&huart3, (uint8_t*)msg_mag, strlen(msg_mag), HAL_MAX_DELAY);
}

static float angle_diff_deg(float a, float b)
{
    float diff = a - b;

    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    return diff;
}


int main(void) {
	MFX_knobs_t knobs;
    HAL_Init();
    MPU_Config();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CRC_Init();
    MX_USART3_UART_Init();

    sensors_init();

    MotionFX_initialize(mfxstate);
    MotionFX_getKnobs(mfxstate, &knobs);

	uint32_t last_tick = HAL_GetTick();

    /* Sampling period: 10 ms */
    knobs.acc_orientation[0]  = 's';
    knobs.acc_orientation[1]  = 'e';
    knobs.acc_orientation[2]  = 'u';

    knobs.gyro_orientation[0] = 's';
    knobs.gyro_orientation[1] = 'e';
    knobs.gyro_orientation[2] = 'u';

    knobs.mag_orientation[0] = 'n';
    knobs.mag_orientation[1] = 'e';
    knobs.mag_orientation[2] = 'd';

    knobs.output_type = MFX_ENGINE_9X;
    knobs.modx = 1;

    MotionFX_setKnobs(mfxstate, &knobs);
    MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
    MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);

    BSP_LED_Init(LED_GREEN);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    last_tick = HAL_GetTick();

    while (1) {
    	const float delta_time = 0.01f; // 100hz

    	if(BSP_PB_GetState(BUTTON_USER)) {
    	    MotionFX_MagCal_init(10, 1);
    	    start_mag_cal();
    	    yaw_reference_set = 0;
    	    last_tick = HAL_GetTick();

    	}

        read_insert_sensors();

        // delta for kalman filter

		MotionFX_propagate(mfxstate, &mfx_output, &mfx_input, &delta_time);
		MotionFX_update(mfxstate, &mfx_output, &mfx_input, &delta_time, NULL);

        // subtracting the inizial yaw to obtain the real yaw
		if (!yaw_reference_set && mfx_output.headingErr < 5.0f) {
		        initial_yaw = mfx_output.rotation[0];
		        yaw_reference_set = 1;
		    }
		relative_yaw = angle_diff_deg(mfx_output.rotation[0], initial_yaw);


		if (now - last_print_tick >= PRINT_INTERVAL){
			last_print_tick = now;
			char msg[256];
			sprintf(msg,
				"Accel: X=%.3f Y=%.3f Z=%.3f | "
				"Combined Acc: X=%.3f Y=%.3f Z=%.3f | "
				"Mag: X=%.3f Y=%.3f Z=%.3f | "
				"Orientation: Relative Yaw=%.2f Pitch=%.2f Roll=%.2f | "
				"Heading: %.2f | Heading Error: %.2f \r\n\r\n",
				mfx_input.acc[0], mfx_input.acc[1], mfx_input.acc[2],
				mfx_output.linear_acceleration[0], mfx_output.linear_acceleration[1], mfx_output.linear_acceleration[2],
				mfx_input.mag[0], mfx_input.mag[1], mfx_input.mag[2],
				// from the sensor data the roll and pitch angles are switched with respect to the comment of the structure in the library
				relative_yaw, mfx_output.rotation[1], mfx_output.rotation[2],
				mfx_output.heading, mfx_output.headingErr
			);

			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @retval None
  */
static void MX_CRC_Init(void)
{
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* MPU Configuration */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
