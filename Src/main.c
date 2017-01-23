/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "pid.h"
#include "vl53l0x_user_port.h"
#include "hardware.h"
#include "communication.h"
#include "motion.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t KTIR_data[] = { 0, 0, 0, 0, 0 };
//VLX values
uint32_t measurement = 0;
uint32_t measurement2 = 0;
uint8_t dataReady = 0;

enum robot_State {
	WAIT, SEARCH, FIGHT, BACK
};
static enum robot_State state = WAIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
VL53L0X_Error VL53L0X_init(VL53L0X_Dev_t *pMyDevice);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	HAL_Init_Peripherals();
	/* USER CODE BEGIN 2 */

	//Start UART monitoring for 4 bytes
	HAL_UART_Receive_IT(&huart2, Received, 4);

	//START ADC and DMA reading KTIR sensors
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) KTIR_data, 5);

	//Initilize motors control
	Motion_init();

	//VLX SOFTWARE
	uint8_t stat;
	uint8_t data;

	//Czujnik 1


	HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, 1); //wylacz 2
	HAL_Delay(50);

	//Czujnik 1


	stat = HAL_I2C_Mem_Read(&hi2c1, 0x52, 0xC1,
	I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
	// Initialize
	pMyDevice->I2cDevAddr = (0b0101001 << 1);
	pMyDevice->I2cDevAddr = 0x52;
	pMyDevice->comms_type = 1;
	pMyDevice->comms_speed_khz = 100;

	////////////////////// VL53L0X initialization and test /////////////////////////
	////////// Interface test
	if (Status == VL53L0X_ERROR_NONE) {
		status_int = VL53L0X_GetVersion(pVersion);
		if (status_int != 0) {
			Status = VL53L0X_ERROR_CONTROL_INTERFACE;

		}
	}

	if (Status == VL53L0X_ERROR_NONE) {

		pVersion->major;
		pVersion->minor;
		pVersion->build;

	}
	////Change adress
	if (Status == VL53L0X_ERROR_NONE) {
		VL53L0X_SetDeviceAddress(pMyDevice, 0x54);
		pMyDevice->I2cDevAddr = 0x54;

	}
	//// device test
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_DataInit(&MyDevice);
		HAL_Delay(50);

	}

	////////// Device test
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
	}
	///device init
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_init(pMyDevice);
	}

	HAL_Delay(50);
	HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, 0);		//wylacz 2
	HAL_Delay(50);

	//Czujnik 2



	pRangingMeasurementData2->RangeMilliMeter = 0;

	//pMyDevice->I2cDevAddr = (0b0101001 << 1);
	pMyDevice2->I2cDevAddr = 0x52;
	pMyDevice2->comms_type = 1;
	pMyDevice2->comms_speed_khz = 100;

	////////////////////// VL53L0X initialization and test /////////////////////////
	////////// Interface test
	if (Status2 == VL53L0X_ERROR_NONE) {
		status_int2 = VL53L0X_GetVersion(pVersion2);
		if (status_int2 != 0) {
			Status2 = VL53L0X_ERROR_CONTROL_INTERFACE;

		}
	}

	//// device test
	if (Status2 == VL53L0X_ERROR_NONE) {
		Status2 = VL53L0X_DataInit(&MyDevice2);
		HAL_Delay(50);

	}

	////////// Device test
	if (Status2 == VL53L0X_ERROR_NONE) {
		Status2 = VL53L0X_GetDeviceInfo(&MyDevice2, &DeviceInfo2);
	}

	///device init
	if (Status2 == VL53L0X_ERROR_NONE) {
		Status2 = VL53L0X_init(pMyDevice2);
	}
	HAL_Delay(50);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		VL53L0X_GetMeasurementDataReady(pMyDevice, &dataReady);
		if (Status == VL53L0X_ERROR_NONE && dataReady == 1) {
			Status = VL53L0X_GetRangingMeasurementData(pMyDevice,
					pRangingMeasurementData);
			measurement = pRangingMeasurementData->RangeMilliMeter;

			VL53L0X_ClearInterruptMask(pMyDevice,
			VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
			dataReady = 0;
		}

		VL53L0X_GetMeasurementDataReady(pMyDevice2, &dataReady);
		if (Status2 == VL53L0X_ERROR_NONE && dataReady == 1) {
			Status2 = VL53L0X_GetRangingMeasurementData(pMyDevice2,
					pRangingMeasurementData2);
			measurement2 = pRangingMeasurementData2->RangeMilliMeter;

			VL53L0X_ClearInterruptMask(pMyDevice2,
			VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
			dataReady = 0;
		}
		switch (state) {
		case WAIT:
			reqSpeedRight = 0;
			reqSpeedLeft = 0;

			break;
		case SEARCH:
			reqSpeedRight = 8;

			if (measurement < 150 && measurement2 < 150)
				state = FIGHT;
			if (KTIR_data[3] < 1000) {
				state = BACK;
			}
			break;
		case FIGHT:
			reqSpeedLeft = 8;
			reqSpeedRight = 8;
			if (KTIR_data[3] < 1000) {
				state = BACK;
			}
			break;
		case BACK:
			reqSpeedLeft = -8;
			reqSpeedRight = -8;
			HAL_Delay(600);
			state = SEARCH;

			break;
		}

	}
	/* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) { // Je¿eli przerwanie pochodzi od timera 10
		Motion_tick();

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BTN1_Pin) {
		if (state != WAIT)
			state = WAIT;
		else
			state = SEARCH;

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	uint8_t Data[4]; // Tablica przechowujaca wysylana wiadomosc.

	int status1, status2;
	status1 = memcmp("STOP", Received, UART2_RX_bufferlen);
	status2 = memcmp("STRT", Received, UART2_RX_bufferlen);

	if (status1 == 0)
		state = WAIT;
	if (status2 == 0)
		state = SEARCH;

	//sprintf((char*) Data, "Command acknowledged: %s",Received);
	print("Command acknowledged\r\n"); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	HAL_UART_Receive_IT(&huart2, (uint8_t*) Received, UART2_RX_bufferlen); // Ponowne w³¹czenie nas³uchiwania

}

VL53L0X_Error VL53L0X_init(VL53L0X_Dev_t *pMyDevice) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	uint32_t refSpadCount = 6;
	uint8_t isApertureSpads = 1;
	uint8_t VhvSettings = 30;
	uint8_t PhaseCal = 3;		//1

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_StaticInit(pMyDevice); // function allows to load device settings specific for a given use //case.
	}

//	if(Status == VL53L0X_ERROR_NONE)
//	    {
//
//	        Status = VL53L0X_PerformRefCalibration(pMyDevice,
//	        		&VhvSettings, &PhaseCal); // Device Initialization
//
//	    }
//
//	  if(Status == VL53L0X_ERROR_NONE)
//	    {
//
//	        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
//	        		&refSpadCount, &isApertureSpads); // Device Initialization
//
//	    }

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetReferenceSpads(pMyDevice, refSpadCount,
				isApertureSpads); // czy jest w otworze
	}

	if (Status == VL53L0X_ERROR_NONE) {

		Status = VL53L0X_SetRefCalibration(pMyDevice, VhvSettings, PhaseCal); // Device Initialization, kalibruj diode

	}

	if (Status == VL53L0X_ERROR_NONE) {

		Status = VL53L0X_SetDeviceMode(pMyDevice,
		VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode

	}
//	//wlacz limity parametrow pomiaru
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
				(FixPoint1616_t) (0.1 * 65536));
	}
	//przesuniecie fazowe w mm
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(pMyDevice,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (60 * 65536));
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
		VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18); ///18
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
		VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14); //14
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
				40000);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetGpioConfig(pMyDevice, 0,
		VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
		VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
		VL53L0X_INTERRUPTPOLARITY_HIGH);
	}
	if (Status == VL53L0X_ERROR_NONE) {

		Status = VL53L0X_StartMeasurement(pMyDevice);

	}

	return Status;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		print("Unidentified error. Please restart.\r\n");
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
