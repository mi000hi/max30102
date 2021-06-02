/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../../Drivers/max30102/max30102.h"

#include "../../Drivers/sx1508b/sx1508b.h"
#include "../../Drivers/sx1508b/sx1508b_default.h"
#include "../../Drivers/sx1508b/sx1508b_core.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint16_t MX30102_PIN_INTERRUPT = 2; // pin 2 on KHC GPIO extender

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

max30102_t max30102_configs;
sx1508b_t sx1508b_configs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* User defined read/write functions for drivers */
HAL_StatusTypeDef i2c_read(uint8_t dev, uint8_t reg, uint8_t *buf,
		uint8_t bytes);
HAL_StatusTypeDef i2c_read(uint8_t dev, uint8_t reg, uint8_t *buf,
		uint8_t bytes);

void max30102_read(uint8_t reg, uint8_t *buf, uint8_t bytes);
void max30102_write(uint8_t reg, uint8_t *buf, uint8_t bytes);
void max30102_readInterruptPin(uint8_t *buf);

void sx1508b_read(uint8_t reg, uint8_t *buf, uint8_t bytes);
void sx1508b_write(uint8_t reg, uint8_t *buf, uint8_t bytes);

/* User defined functions */
void flashLED(uint32_t duration);
void sx1508b_configure();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI2_Init();
	MX_I2C3_Init();
	/* USER CODE BEGIN 2 */

	/* Hand-over function pointer for sensor communication */
	max30102_configs.comm.read = max30102_read;
	max30102_configs.comm.write = max30102_write;
	max30102_configs.comm.readInterruptPin = max30102_readInterruptPin;

	sx1508b_configs.comm.read = sx1508b_read;
	sx1508b_configs.comm.write = sx1508b_write;

	/* setup max30102 heart rate sensor */
	max30102_setup_communication(&max30102_configs);

	/* setup sx1508b gipo extender and check all configs */
	sx1508b_setup_communication(&sx1508b_configs);
	sx1508b_configure();

	flashLED(100);

	/* read from the FIFO queue */
	max30102_setMeasurementMode(MAX30102_MODE_HEART_RATE);
	max30102_setupForMeasurement();

	// measure for one second
	uint8_t buffer[32] = { 0 };
	uint32_t endTime = HAL_GetTick() + 1000;
	while(HAL_GetTick() < endTime) {
		max30102_waitAndGetHeartrateSamples(buffer);
	}

	// TODO: convert data to heartrate count
	// TODO: put sensor back to sleep mode


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
	PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x10909CEC;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_1LINE;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BLUE_RST_GPIO_Port, BLUE_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, CS_AG_Pin | BLUE_CS_Pin | CS_M_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_P_GPIO_Port, CS_P_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_A_GPIO_Port, CS_A_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BLUE_RST_Pin */
	GPIO_InitStruct.Pin = BLUE_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BLUE_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_BUTTON_Pin */
	GPIO_InitStruct.Pin = USER_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : CS_AG_Pin BLUE_CS_Pin CS_M_Pin */
	GPIO_InitStruct.Pin = CS_AG_Pin | BLUE_CS_Pin | CS_M_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : CS_P_Pin */
	GPIO_InitStruct.Pin = CS_P_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_P_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BLUE_IRQ_Pin */
	GPIO_InitStruct.Pin = BLUE_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BLUE_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CS_A_Pin */
	GPIO_InitStruct.Pin = CS_A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_A_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * activates the LED for the specified amount of ms
 * @param duration		the duration of the LED to be ON, in ms
 */
void flashLED(uint32_t duration) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(duration);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

/**
 * configure the GPIO extender here
 */
void sx1508b_configure() {

	uint8_t reg;

	// direction register -- set all gpio pins as outputs
	reg = 0x00;
	sx1508b_write(SX1508B_REG_DIR, &reg, 1);

	// activate PWM
//	sx1508b_configure_clock(0b0100);
//	sx1508b_configure_pwm(0b100);
}

/**
 * @brief i2c interface read function for XXX
 * @param uint8_t dev: 7 bit device address from which data will be read
 * @param uint8_t reg: sensor register address from which the data will be read
 * @param uint8_t* buf: pointer to the buffer in which read data will be written
 * @param uint8_t* bytes: number of bytes which will be read
 */
HAL_StatusTypeDef i2c_read(uint8_t dev, uint8_t reg, uint8_t *buf,
		uint8_t bytes) {

	// convert 7 bit device address to 16 bit address
	uint16_t DEVICE_ADDR = ((uint16_t) dev) << 1;

	// read data from device memory
	return HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf,
			bytes, 1000);
}

/**
 * @brief i2c interface write function for XXX
 * @param uint8_t dev: 7 bit device address to which data will be written
 * @param uint8_t reg: sensor register address to which the buffer will be written
 * @param uint8_t* buf: pointer to the buffer which contains the writing data
 * @param uint8_t* bytes: number of bytes which will be written
 */
HAL_StatusTypeDef i2c_write(uint8_t dev, uint8_t reg, uint8_t *buf,
		uint8_t bytes) {

	// convert 7 bit device address to 16 bit address
	uint16_t DEVICE_ADDR = ((uint16_t) dev) << 1;

	// write data to device memory
	return HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			buf, bytes, 1000);

}

/**
 * @brief User-defined i2c read function for MAX30102
 * @param uint8_t reg: 	gpio extender register address to which the buffer will be written
 * @param uint8_t* buf: 	pointer to the buffer in which read data will be written
 * @retval None
 */
void max30102_read(uint8_t reg, uint8_t *buf, uint8_t bytes) {

	HAL_StatusTypeDef err = HAL_OK;
	err = i2c_read(MAX30102_ADDRESS_BASE, reg, buf, bytes);

	if (err != HAL_OK) {
		// TODO
	}

}

/**
 * reads the value of the interrupt pin for the MAX30102 sensor
 *
 * @param buf	the buffer where the pin value is written to
 */
void max30102_readInterruptPin(uint8_t *buf) {
	// TODO: uncomment once sx1508b driver is added
	sx1508b_read_pin(MX30102_PIN_INTERRUPT, buf);
}

/**
 * @brief User-defined i2c write function for MAX30102
 * @param uint8_t reg: 	gpio extender register address to which the buffer will be written
 * @param uint8_t* buf: 	pointer to the buffer which contains the writing data
 * @retval None
 */
void max30102_write(uint8_t reg, uint8_t *buf, uint8_t bytes) {

	HAL_StatusTypeDef err = HAL_OK;
	err = i2c_write(MAX30102_ADDRESS_BASE, reg, buf, bytes);

	if (err != HAL_OK) {
		// TODO
	}

}

/**
 * @brief User-defined i2c read function for SX1508B
 * @param uint8_t reg: 	gpio extender register address to which the buffer will be written
 * @param uint8_t* buf: 	pointer to the buffer in which read data will be written
 * @retval None
 */
void sx1508b_read(uint8_t reg, uint8_t *buf, uint8_t bytes) {

	HAL_StatusTypeDef err = HAL_OK;
	err = i2c_read(SX1508B_ADDRESS_BASE << 2 | 0b00, reg, buf, bytes);

	if (err != HAL_OK) {
		// TODO
	}

}

/**
 * @brief User-defined i2c write function for SX1508B
 * @param uint8_t reg: 	gpio extender register address to which the buffer will be written
 * @param uint8_t* buf: 	pointer to the buffer which contains the writing data
 * @retval None
 */
void sx1508b_write(uint8_t reg, uint8_t *buf, uint8_t bytes) {

	HAL_StatusTypeDef err = HAL_OK;
	err = i2c_write(SX1508B_ADDRESS_BASE << 2 | 0b00, reg, buf, bytes);

	if (err != HAL_OK) {
		// TODO
	}

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
