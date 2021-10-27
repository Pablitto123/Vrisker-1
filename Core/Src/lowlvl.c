/*
 * lowlvl.c
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#include <lowlvl.h>

extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;

extern I2C_HandleTypeDef hi2c1;

uint32_t raw_voltage_adc;

uint8_t result1, result2, result3, result4;
uint64_t write_duration;
uint64_t read_duration;
uint64_t start_time;
volatile uint32_t eeprom_cycle_counter = 0;
volatile uint8_t eeprom_data = 0xff;
uint32_t eeprom_test_counter = 1000000000;
uint32_t eeprom_test_counter2 = 0;
uint16_t eeprom_address;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern int32_t actual_left_duty;
extern int32_t actual_right_duty;
int last_left_set = 0, last_right_set = 0;

extern TIM_HandleTypeDef htim14;
uint8_t ledsEnabledMask;
uint8_t timer;
uint8_t ldpwms[4] = { LED2_OUTPUT_PWM / 10, LED3_OUTPUT_PWM / 10,
		LED4_OUTPUT_PWM / 10, LED5_OUTPUT_PWM / 10 };

int32_t actual_left_duty = 0;
int32_t actual_right_duty = 0;
extern TIM_HandleTypeDef htim12;

extern I2C_HandleTypeDef hi2c1;
int mpu_is_initialized;
int dmpIntDataReady;
int16_t receivedMPUData[7];

extern UART_HandleTypeDef huart3;
UARTDMA_HandleTypeDef huartdma;
char ParseBuffer[16];
char TransmitBuffer[UART_SENDING_BUFFER_SIZE];
int transmitBufferSize = 0;

extern I2C_HandleTypeDef hi2c1;
int dmaI2CDataReceived = 1;
int dmaI2CDataTransmited = 1;
VL53L0X_RangingMeasurementData_t RangingData[5];
VL53L0X_Dev_t Dev[5];
volatile int TOF_DATA_READ[5] = { FALSE, FALSE, FALSE, FALSE, FALSE };
uint16_t DIST[5] = { 0, 0, 0, 0, 0 };
int vls_initialized = FALSE;

uint8_t start_task_list[14];
float battery_voltage = 0;
int bat_debug_flag = 0;

extern TIM_HandleTypeDef htim4, htim6, htim7, htim9, htim10;
extern TIM_HandleTypeDef htim13;
extern float reference_omega;
extern float reference_velocity;
extern float left_velocity;
extern float right_velocity;
int raw_read_velocity = 0, raw_read_omega = 0;

extern float vP;
extern float vI;
extern float vD;
extern float wP;
extern float wI;
extern float wD;

int raw_vP = 0,raw_vI = 0,raw_vD = 0,raw_wP = 0,raw_wI = 0,raw_wD = 0;

extern int pid_print_flag;

// INTERNAL MEMORY SECTION
uint16_t startup_counter_addr = 0x0000;
uint32_t startup_counter_data = 0;
uint16_t vp_addr = 0x0004;
uint32_t vp_data = 0;
uint16_t vi_addr = 0x0008;
uint32_t vi_data = 0;
uint16_t vd_addr = 0x000C;
uint32_t vd_data = 0;
uint16_t wp_addr = 0x0010;
uint32_t wp_data = 0;
uint16_t wi_addr = 0x0014;
uint32_t wi_data = 0;
uint16_t wd_addr = 0x0018;
uint32_t wd_data = 0;

int request2savedata = 0;

void runBatteryLevelADC(uint32_t *variable) {
	HAL_ADC_Start_DMA(&hadc3, variable, 1);
}

void performLowVoltageAction() {
	HAL_ADC_Stop_DMA(&hadc3);
	TIM12->CCR1 = 0;
	TIM12->CCR2 = 0;
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);

	//stop uart
	//stop sheduler

}

void eepromMemoryDump(uint32_t timeout) {
	//zczytanie calej pamieci
	for (uint16_t page_count = 0x00; page_count < 0x200; page_count++)
		for (uint16_t word_count = 0x00; word_count < 0x40; word_count++) {
			eeprom_address = (page_count << 6) + word_count;
			while (HAL_I2C_Mem_Read_DMA(&hi2c1, EEPROM_DEV_ADDRESS,
					eeprom_address, 2, (uint8_t*) &eeprom_data, 1) != HAL_OK) {
				if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
					Error_Handler();
				}
				printf("I2C Connection Error\n");
			}
			HAL_Delay(DELAY_EEPROM_CYCLE_READ);
			printf("%.1fk 0x%x: 0x%x\n", (double) (page_count * 256.0f / 0x200),
					eeprom_address, eeprom_data);
			ledToggle(4);
			HAL_Delay(timeout);
		}
	printf("EEPROM 256k dump DONE\n");
	HAL_Delay(5000);
}

/*
 * WARNING: AVOID using this FCN
 * INFO: BKUP required
 * */
void eepromFullMemoryErase() {
	//czyszczenie pamieci
	for (uint16_t page_count = 0x00; page_count < 0x200; page_count++)
		for (uint16_t word_count = 0x00; word_count < 0x40; word_count++) {
			eeprom_address = (page_count << 6) + word_count;
			while (HAL_I2C_Mem_Write(&hi2c1, EEPROM_DEV_ADDRESS, eeprom_address,
					2, (uint8_t*) &eeprom_data, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
				if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
					Error_Handler();
				}
				HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
			}
			HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
			printf("%.1lfk\n", (double) (page_count * 256.0 / 0x200));
		}
	printf("EEPROM 256k erase DONE\n");
	HAL_Delay(5000);
}

uint32_t eepromRead4Bytes(uint16_t addr) {

	while (HAL_I2C_Mem_Read(&hi2c1, EEPROM_DEV_ADDRESS, addr, 2,
			(uint8_t*) &result1, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		HAL_Delay(DELAY_EEPROM_CYCLE_READ);
	}
	HAL_Delay(DELAY_EEPROM_CYCLE_READ);
	while (HAL_I2C_Mem_Read(&hi2c1, EEPROM_DEV_ADDRESS, addr + 1, 2,
			(uint8_t*) &result2, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		HAL_Delay(DELAY_EEPROM_CYCLE_READ);
	}
	HAL_Delay(DELAY_EEPROM_CYCLE_READ);
	while (HAL_I2C_Mem_Read(&hi2c1, EEPROM_DEV_ADDRESS, addr + 2, 2,
			(uint8_t*) &result3, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		HAL_Delay(DELAY_EEPROM_CYCLE_READ);
	}
	HAL_Delay(DELAY_EEPROM_CYCLE_READ);
	while (HAL_I2C_Mem_Read(&hi2c1, EEPROM_DEV_ADDRESS, addr + 3, 2,
			(uint8_t*) &result4, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		HAL_Delay(DELAY_EEPROM_CYCLE_READ);
	}
	HAL_Delay(DELAY_EEPROM_CYCLE_READ);
	return (result1 << 24 | result2 << 16 | result3 << 8 | result4);
}

void eepromSave4Bytes(uint16_t *addr, uint32_t *data) {
	result1 = (*data >> 24), result2 = (*data >> 16), result3 = (*data >> 8), result4 =
			*data;

	while (HAL_I2C_Mem_Write(&hi2c1, EEPROM_DEV_ADDRESS, *addr, 2,
			(uint8_t*) &result1, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
	}
	//{}else printf("I2C Connection Error1-Write\n");
	HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
	while (HAL_I2C_Mem_Write(&hi2c1, EEPROM_DEV_ADDRESS, *addr + 1, 2,
			(uint8_t*) &result2, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
	}
	//{}else printf("I2C Connection Error2-Write\n");
	HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
	while (HAL_I2C_Mem_Write(&hi2c1, EEPROM_DEV_ADDRESS, *addr + 2, 2,
			(uint8_t*) &result3, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
	}
	//{}else printf("I2C Connection Error3-Write\n");
	HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
	while (HAL_I2C_Mem_Write(&hi2c1, EEPROM_DEV_ADDRESS, *addr + 3, 2,
			(uint8_t*) &result4, 1, EEPROM_I2C_TIMEOUT) != HAL_OK) {
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
			Error_Handler();
		}
		HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
	}
	//{}else printf("I2C Connection Error4-Write\n");
	HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
}

void eepromMemoryTestRW() {
	start_time = HAL_GetTick();
	//zapis
	for (uint16_t page_count = 0x00; page_count < 0x200; page_count++)
		for (uint16_t word_count = 0x00; word_count < 0x40; word_count += 4) {
			eeprom_address = (page_count << 6) + word_count;
			eepromSave4Bytes(&eeprom_address, &eeprom_test_counter);
			HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
			if (page_count % 2 == 0 && word_count % 60 == 0) {
				printf("%.1lfk\n", (double) (page_count * 256.0 / 0x200));
			}
			HAL_Delay(DELAY_EEPROM_CYCLE_WRITE);
			eeprom_test_counter++;
		}
	write_duration = HAL_GetTick() - start_time;
	printf("END\n");
	//odczyt + sprawdzenie
	uint32_t i = 1000000000;
	start_time = HAL_GetTick();
	for (uint16_t page_count = 0x00; page_count < 0x200; page_count++)
		for (uint16_t word_count = 0x00; word_count < 0x40; word_count += 4) {
			eeprom_address = (page_count << 6) + word_count;
			eeprom_test_counter2 = eepromRead4Bytes(eeprom_address);
			HAL_Delay(DELAY_EEPROM_CYCLE_READ);
			if (page_count % 2 == 0 && word_count % 60 == 0) {
				printf("%.1lfk\n", (double) (page_count * 256.0 / 0x200));
			}
			HAL_Delay(DELAY_EEPROM_CYCLE_READ);
			//sprintf(uart_sending_buffer, "%d %d\n",i,eeprom_test_counter2 );
			//printf((char *) uart_sending_buffer);
			if (eeprom_test_counter2 != i) {
				printf("Bad R/W operation at %x. Expected %ld. Given %ld.\n",
						eeprom_address, i, eeprom_test_counter2);
				HAL_Delay(5000);
				return;
			}
			i++;
		}
	read_duration = HAL_GetTick() - start_time;
	printf(
			"Test passed!\n Stats:\n Write 32 kylobytes in %lld \n Read 32 kylobytes in %lld \n",
			write_duration, read_duration);
	HAL_Delay(5000);
}

void eepromMemoryTestErase() {
	for (uint16_t page_count = 0x00; page_count < 0x200; page_count++)
		for (uint16_t word_count = 0x00; word_count < 0x40; word_count++) {
			eeprom_address = (page_count << 6) + word_count;
			while (HAL_I2C_Mem_Read_DMA(&hi2c1, EEPROM_DEV_ADDRESS,
					eeprom_address, 2, (uint8_t*) &eeprom_data, 1) != HAL_OK)
				if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
					Error_Handler();
				}
			HAL_Delay(DELAY_EEPROM_CYCLE_READ);
			if (page_count % 2 == 0 && word_count % 63 == 0) {
				printf("%.1lfk\n", (double) (page_count * 256.0 / 0x200));
			}
			if (eeprom_data != 0xff) {
				printf("Erase operation failed at %x. Given %d.\n",
						eeprom_address, eeprom_data);
				HAL_Delay(5000);
				return;
			}
		}
	printf("Test passed");
	HAL_Delay(5000);
}

GPIO_PinState BTN_IsPressed(uint8_t btnid) {
	if (btnid == 1)
		return !HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin);
	else if (btnid == 2)
		return !HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin);
	return GPIO_PIN_RESET;
}

void encodersInit() {
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	resetLeftTickCount();
	resetRightTickCount();
}

int32_t getLeftTickCount() {
	int32_t product = (int) (TIM2->CNT) - last_left_set;
	return -(product > 32000 ? product - 65535 : product);
}
int32_t getRightTickCount() {
	int32_t product = (int) (TIM3->CNT) - last_right_set;
	return -(product > 32000 ? product - 65535 : product);
}

void setEncoderCount(int id, int16_t value){
	switch(id){
		case 1:
			TIM2->CNT = value;
			break;
		case 2:
			TIM3->CNT = value;
			break;
	}
}

void resetLeftTickCount() {
	if (actual_left_duty >= 0) {
		TIM2->CNT = 16000;
		last_left_set = 16000;
	} else {
		TIM2->CNT = 32000;
		last_left_set = 32000;
	}
}

void resetRightTickCount() {
	if (actual_right_duty >= 0) {
		TIM3->CNT = 16000;
		last_right_set = 16000;
	} else {
		TIM3->CNT = 32000;
		last_right_set = 32000;
	}
}

void ledOutputInit() {
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	TIM14->CCR1 = 0;
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
	ledsEnabledMask = 0;
	timer = 0;
}

void ledToggle(uint16_t ledx) {
	switch (ledx) {
	case 1:
		if (TIM14->CCR1 == 0)
			TIM14->CCR1 = LED1_OUTPUT_PWM;
		else
			TIM14->CCR1 = 0;
		break;
	default:
		if (ledsEnabledMask & (1 << ledx))
			ledDisable(ledx);
		else
			ledEnable(ledx);
		break;
	}
}

void ledEnable(uint16_t ledx) {
	switch (ledx) {
	case 1:
		TIM14->CCR1 = LED1_OUTPUT_PWM;
		break;
	default:
		ledsEnabledMask = (1 << ledx) | ledsEnabledMask;
		break;
	}
}

void ledDisable(uint16_t ledx) {
	switch (ledx) {
	case 1:
		TIM14->CCR1 = 0;
		break;
	default:
		// usunięcie bitu na pozycji ledx = 3 ledsEnabledMask: 010101
		// usuwamy 3
		// 010101 ^ 001000 -> 011101
		// 011101 & 010101 -> 010101 nie było bitu i nadal go nie ma ;-)
		// usuwamy 2
		// 010101 ^ 000100 -> 010001
		// 010001 & 010101 -> 010001 był bit i nie ma ;-)
		ledsEnabledMask = (ledsEnabledMask ^ (1 << ledx)) & ledsEnabledMask;
		break;
	}
}

void generateLEDPWMCallback(void) {
	// FIXME Ledy jarzyły za jasno, można było zmienić rezystory, ale stwierdziłem, że wyreguluję to softem
	timer = (timer + 1) % 100;
	//PWM PROMILES
	//PWM SIGNAL 100 %% _________-
	//PWM SIGNAL 200 %% ________--
	//PWM SIGNAL 300 %% _______---
	//PWM SIGNAL 400 %% ______----
	//PWM SIGNAL 500 %% _____-----

	if (timer == 0) {
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		return;
	} else
		//led 2 - 5
		for (int led = 2; led < 6; led++)
			if (ledsEnabledMask & (1 << led))
				if (99 - timer == ldpwms[led - 2]) {
					switch (led) {
					case 2:
						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
								GPIO_PIN_SET);
						break;
					case 3:
						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,
								GPIO_PIN_SET);
						break;
					case 4:
						HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,
								GPIO_PIN_SET);
						break;
					case 5:
						HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin,
								GPIO_PIN_SET);
						break;
					}
				}
}

void motorsPWMInit() {
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	TIM12->CCR1 = 0;
	TIM12->CCR2 = 0;
}

void motorLeftSetPWM(int32_t duty) {
	if (duty * actual_left_duty <= 0) {
		HAL_GPIO_WritePin(AFOR_GPIO_Port, AFOR_Pin,
				duty >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(ABAC_GPIO_Port, ABAC_Pin,
				duty >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
	TIM12->CCR1 = duty > 0 ? duty : -duty;
	actual_left_duty = duty;
}







































































































































































































































































































































































































































































































































































































































































































































































































































































































































































void motorRightSetPWM(int32_t duty) {
	if (duty * actual_right_duty <= 0) {
		HAL_GPIO_WritePin(BFOR_GPIO_Port, BFOR_Pin,
				duty >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BBAC_GPIO_Port, BBAC_Pin,
				duty >= 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
	TIM12->CCR2 = duty > 0 ? duty : -duty;
	actual_right_duty = duty;
}

int motorLeftGetPWM() {
	return actual_left_duty;
}
int motorRightGetPWM() {
	return actual_right_duty;
}

void IMU_MPU6050_Init(void) {
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1 << 7); //reset the whole module first
	HAL_Delay(50);    //wait for 50ms for the gyro to stable
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
			MPU6050_CLOCK_PLL_ZGYRO);    //PLL with Z axis gyroscope reference
	HAL_Delay(DELAY_IMU_CYCLE_WRITE);
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01); //DLPF_CFG = 1: Fs=1khz; bandwidth=42hz
	HAL_Delay(DELAY_IMU_CYCLE_WRITE);
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x01); //500Hz sample rate ~ 2ms
	HAL_Delay(DELAY_IMU_CYCLE_WRITE);
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,
			MPU6050_GYRO_FS_2000);    //Gyro full scale setting
	HAL_Delay(DELAY_IMU_CYCLE_WRITE);
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG,
			MPU6050_ACCEL_FS_2);    //Accel full scale setting
	HAL_Delay(DELAY_IMU_CYCLE_WRITE);
	//MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1<<4);        //interrupt status bits are cleared on any read operation
	HAL_Delay(DELAY_IMU_CYCLE_WRITE);
	//MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 1<<0);        //interupt occurs when data is ready. The interupt routine is in the receiver.c file.
	HAL_Delay(DELAY_IMU_CYCLE_WRITE);
	MPU6050_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SIGNAL_PATH_RESET, 0x07); //reset gyro and accel sensor
	HAL_Delay(50); //wait for 50ms for the gyro to stable
	//dmpIntDataReady = FALSE;
	//HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	mpu_is_initialized = TRUE;
}

void MPU6050_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data) {
	uint8_t tmp;
	tmp = data;
	MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

int getMPUDataReadyStatus() {
	return dmpIntDataReady;
}
void setMPUDataReadyStatus(int enable) {
	dmpIntDataReady = enable;
}

void IMU_MPU6050_ReadRaw(int16_t *buff) {
	uint8_t tmpBuffer[14];
	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer,
			MPU6050_RA_ACCEL_XOUT_H, 14);
	/* Get acceleration */
	for (int i = 0; i < 3; i++)
		buff[i] = ((uint16_t) ((uint16_t) tmpBuffer[2 * i] << 8)
				+ tmpBuffer[2 * i + 1]);
	/* Get temperature */
	buff[3] = (int16_t) ((tmpBuffer[6] << 8) | tmpBuffer[7]) / 340 + 37;
	/* Get Angular rate */
	for (int i = 4; i < 7; i++)
		buff[i] = ((int16_t) ((uint16_t) tmpBuffer[2 * i] << 8)
				+ tmpBuffer[2 * i + 1]);
}

void MPUInterruptHandler(void) {
	//setMPUDataReadyStatus(TRUE);
	IMU_MPU6050_ReadRaw(receivedMPUData);
}

void sendChar(char c) {
	if (transmitBufferSize < UART_SENDING_BUFFER_SIZE)
		TransmitBuffer[transmitBufferSize++] = c;
}

int __io_putchar(int ch) {
	if (ch == '\n')
		sendChar('\r');
	sendChar(ch);
	return ch;
}

void UARTDMA_UartIrqHandler(UARTDMA_HandleTypeDef *huartdma) {
	if (huartdma->huart->Instance->SR & UART_FLAG_IDLE) // Check if Idle flag is set
	{
		volatile uint32_t tmp;
		tmp = huartdma->huart->Instance->SR;             // Read status register
		tmp = huartdma->huart->Instance->DR;               // Read data register
		huartdma->huart->hdmarx->Instance->CR &= ~DMA_SxCR_EN; // Disable DMA - it will force Transfer Complete interrupt if it's enabled
		tmp = tmp; // For unused warning
	}
}

void UARTDMA_DmaIrqHandler(UARTDMA_HandleTypeDef *huartdma) {
	uint8_t *UartBufferPointer, *DmaBufferPointer;
	uint32_t Length;
	uint16_t i, TempHead;

	typedef struct {
		__IO uint32_t ISR;   // DMA interrupt status register
		__IO uint32_t Reserved0;
		__IO uint32_t IFCR;  // DMA interrupt flag clear register
	} DMA_Base_Registers;

	DMA_Base_Registers *DmaRegisters =
			(DMA_Base_Registers*) huartdma->huart->hdmarx->StreamBaseAddress; // Take registers base address

	if (__HAL_DMA_GET_IT_SOURCE(huartdma->huart->hdmarx, DMA_IT_TC) != RESET) // Check if interrupt source is Transfer Complete
			{
		DmaRegisters->IFCR = DMA_FLAG_TCIF0_4
				<< huartdma->huart->hdmarx->StreamIndex; // Clear Transfer Complete flag

		Length = DMA_RX_BUFFER_SIZE - huartdma->huart->hdmarx->Instance->NDTR; // Get the Length of transfered data

		UartBufferPointer = huartdma->UART_RX_Buffer;
		DmaBufferPointer = huartdma->DMA_RX_Buffer;

		// Write received data for UART main buffer - circular buffer
		for (i = 0; i < Length; i++) {
			TempHead = (huartdma->UartBufferHead + 1)
					% UART_RECEIVING_BUFFER_SIZE;
			if (TempHead == huartdma->UartBufferTail) {
				huartdma->UartBufferHead = huartdma->UartBufferTail; // No room for new data
			} else {
				UartBufferPointer[TempHead] = DmaBufferPointer[i];
				if (UartBufferPointer[TempHead] == '\n') {
					huartdma->UartBufferLines++;
				}
				huartdma->UartBufferHead = TempHead;
			}
		}

		DmaRegisters->IFCR = 0x3FU << huartdma->huart->hdmarx->StreamIndex; // Clear all interrupts
		huartdma->huart->hdmarx->Instance->M0AR =
				(uint32_t) huartdma->DMA_RX_Buffer; // Set memory address for DMA again
		huartdma->huart->hdmarx->Instance->NDTR = DMA_RX_BUFFER_SIZE; // Set number of bytes to receive
		huartdma->huart->hdmarx->Instance->CR |= DMA_SxCR_EN; // Start DMA transfer
	}
}

int UARTDMA_GetCharFromBuffer(UARTDMA_HandleTypeDef *huartdma) {
	if (huartdma->UartBufferHead == huartdma->UartBufferTail) {
		return -1; // error - no char to return
	}
	huartdma->UartBufferTail = (huartdma->UartBufferTail + 1)
			% UART_RECEIVING_BUFFER_SIZE;

	return huartdma->UART_RX_Buffer[huartdma->UartBufferTail];
}

uint8_t UARTDMA_IsDataReady(UARTDMA_HandleTypeDef *huartdma) {
	if (huartdma->UartBufferLines)
		return 1;
	else
		return 0;
}

int UARTDMA_GetLineFromBuffer(UARTDMA_HandleTypeDef *huartdma, char *OutBuffer) {
	char TempChar;
	char *LinePointer = OutBuffer;
	if (huartdma->UartBufferLines) {
		while ((TempChar = UARTDMA_GetCharFromBuffer(huartdma))) {
			if (TempChar == '\n') {
				break;
			}
			*LinePointer = TempChar;
			LinePointer++;
		}
		*LinePointer = 0; // end of cstring
		huartdma->UartBufferLines--; // decrement line counter
	}
	return 0;
}

void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart) {
	huartdma->huart = huart;

	__HAL_UART_ENABLE_IT(huartdma->huart, UART_IT_IDLE); // UART Idle Line interrupt
	__HAL_DMA_ENABLE_IT(huartdma->huart->hdmarx, DMA_IT_TC); // UART DMA Transfer Complete interrupt

	HAL_UART_Receive_DMA(huartdma->huart, huartdma->DMA_RX_Buffer,
			DMA_RX_BUFFER_SIZE); // Run DMA for whole DMA buffer

	huartdma->huart->hdmarx->Instance->CR &= ~DMA_SxCR_HTIE; // Disable DMA Half Complete interrupt
}

void uart_rx(void) {
	if (UARTDMA_IsDataReady(&huartdma)) {
		UARTDMA_GetLineFromBuffer(&huartdma, ParseBuffer);
		if (strcmp(ParseBuffer, "A") == 0) {
			ledEnable(4);
		} else if (strcmp(ParseBuffer, "B") == 0) {
			ledDisable(4);
		} else if (strcmp(ParseBuffer, "0") == 0) {
			motorLeftSetPWM(0);
			motorRightSetPWM(0);
		} else if (strcmp(ParseBuffer, "Q") == 0) {
			//not printf in loops
			//debug_flag = 0;
		} else if (strcmp(ParseBuffer, "E") == 0) {
			eepromMemoryDump(5);
			//eepromMemoryTestRW();
			//eepromFullMemoryErase();
			//eepromMemoryTestErase();
			//eepromMemoryDump(5);
		} else if (strcmp(ParseBuffer, "F") == 0) {
			motorLeftSetPWM(200);
			motorRightSetPWM(-100);
			//time_eight = HAL_GetTick();
		} else if (strcmp(ParseBuffer, "R") == 0) {
			HAL_NVIC_SystemReset();
		} else if (strcmp(ParseBuffer, "BAT") == 0) {
			bat_debug_flag = !bat_debug_flag;
		} else if (strcmp(ParseBuffer, "KALMAN") == 0) {
			motorLeftSetPWM(100);
			motorRightSetPWM(100);
			//kalman_flag = 1;
		} else {
			if (ParseBuffer[0] == 'E') {
				//format E,R/W,adress,dane_binarne\n
				//E,W,13,0\n
//		                      sscanf(ParseBuffer,"E,%c,%d,%d\n",&arg1,&arg2,&arg3);
//		                      //printf("%c %d %d\n", arg1,arg2,arg3);
//		                      if(arg1 == 'R'){
//		                          uint32_t b = eepromRead4Bytes(eeprom_task_addr);
//		                          printf("%d\n",b);
//		                          printf("%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n",(b&1)>0,(b&2)>0,(b&4)>0,(b&8)>0,(b&16)>0,(b&32)>0,(b&64)>0,(b&128)>0,(b&256)>0,(b&512)>0,(b&1024)>0,(b&2048)>0,(b&4096)>0,(b&8192)>0);
//		                      }
//		                      else if(arg1 == 'W'){
//		                          eeprom_task_data = eepromRead4Bytes(0x0000);
//		                          if((eeprom_task_data&(1<<arg2)))eeprom_task_data -= (1<<arg2);
//		                          eeprom_task_data = eeprom_task_data | (arg3 << arg2);
//		                          HAL_Delay(EEPROM_I2C_TIMEOUT);
//		                          eepromSave4Bytes(&eeprom_task_addr, &eeprom_task_data);
//		                          HAL_Delay(EEPROM_I2C_TIMEOUT);
//		                          uint32_t b = eepromRead4Bytes(eeprom_task_addr);
//		                          printf("%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n",(b&1)>0,(b&2)>0,(b&4)>0,(b&8)>0,(b&16)>0,(b&32)>0,(b&64)>0,(b&128)>0,(b&256)>0,(b&512)>0,(b&1024)>0,(b&2048)>0,(b&4096)>0,(b&8192)>0);
//		                      }else if(arg1 == 'E'){
//		                          eepromFullMemoryErase();
//		                      }else if(arg1 == 'D'){
//		                          eeprom_task_data = 7173;
//		                          eepromSave4Bytes(&eeprom_task_addr, &eeprom_task_data);
//		                      }
//		                      else if(arg1 == 'I'){
//		                          printf("0-batlvl,1-belfor,2-btnctl,3-dijkst,4-distac\n");
//		                          printf("5-eeprom,6-flofil,7-imuact,8-kalfil,9-lamadt\n");
//		                          printf("10-pidctl,11-uarttx,12-uartrx,13-pathpl\n");
//		                      }
			} else if(ParseBuffer[0] == 'v')
			{
				//sscanf(ParseBuffer, "v%d w%d\n", &raw_read_velocity, &raw_read_omega);
//				sscanf(ParseBuffer, "v%d %d\n", &raw_read_omega, &raw_read_velocity);
//				reference_velocity = (float)raw_read_velocity;
//				reference_omega = (float)raw_read_omega;
//				set_velocity(1, raw_read_velocity);
//				set_velocity(2, raw_read_omega);
				sscanf(ParseBuffer, "v%d %d\n", &raw_read_omega, &raw_read_velocity);
				left_velocity = (float)raw_read_velocity;
				right_velocity = (float)raw_read_omega;
			}
			else if(ParseBuffer[0] == 'p'){
				sscanf(ParseBuffer, "p%d i%d d%d\n", &raw_vP, &raw_vI, &raw_vD);
				vP = (float)raw_vP;
				vI = (float)raw_vI;
				vD = (float)raw_vD;
				// SAVE TO EXT MEM
				vp_data = (uint32_t)raw_vP;
				vi_data = (uint32_t)raw_vI;
				vd_data = (uint32_t)raw_vD;
				request2savedata = 1;
			}
			else if(ParseBuffer[0] == 'P'){
				sscanf(ParseBuffer, "P%d I%d D%d\n", &raw_wP, &raw_wI, &raw_wD);
				wP = (float)raw_wP;
				wI = (float)raw_wI;
				wD = (float)raw_wD;
				// SAVE TO EXT MEM
				wp_data = (uint32_t)raw_wP;
				wi_data = (uint32_t)raw_wI;
				wd_data = (uint32_t)raw_wD;
				request2savedata = 2;
			}
			else if(ParseBuffer[0] == 's' && ParseBuffer[1] == 'h')
			{
				printf("PID I(v) p=%.1f i=%.1f d=%.1f\nPID II(w) p=%.1f i=%.1f d=%.1f\n", vP,vI,vD,wP,wI,wD);
			}
			else if(ParseBuffer[0] == 'd')
			{
				if(pid_print_flag == 0)pid_print_flag = 1;
				else if(pid_print_flag == 1)pid_print_flag = 2;
				else if(pid_print_flag == 2)pid_print_flag = 0;
			}
			else if(ParseBuffer[0] == 'D')
			{
				pid_print_flag = 0;
			}
			else {
				printf("Unknown command %s\n", ParseBuffer);
			}
		}
	}
}

void uart_tx(void) {
	if (huart3.gState == HAL_UART_STATE_READY)
		if (transmitBufferSize > 0) {
			HAL_UART_Transmit_DMA(&huart3, (uint8_t*) TransmitBuffer,
					transmitBufferSize);
			transmitBufferSize = 0;
		}
}

void VL53L0X_Init() {
	uint16_t Id;
	VL53L0X_DisableInterrupts();
	VL53L0X_DeviceError status = 0;
	VL53L0X_ResetAllSensors();
	HAL_Delay(10);
	for (int i = 0; i < 5; i++) {
		VL53L0X_SensorTurnOn(i);
		Dev[i].I2cHandle = &hi2c1;
		Dev[i].I2cDevAddr = 0x52;
		Dev[i].Present = 0;
		Dev[i].Id = 1;
		Dev[i].DevLetter = 'a';
		HAL_Delay(2);
		uint8_t FinalAddress = Dev[i].I2cDevAddr + 2 * (i + 1);
		do {
			/* Set I2C standard mode (400 KHz) before doing the first register access */
			if (status == VL53L0X_ERROR_NONE)
				status = VL53L0X_WrByte(&Dev[i], 0x88, 0x00);
			/* Try to read one register using default 0x52 address */
			status = VL53L0X_RdWord(&Dev[i],
					VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
			HAL_Delay(2);
			if (status) {
				printf("#%d Read id fail\n", i);
				break;
			}
			if (Id == 0xEEAA) {
				/* Sensor is found => Change its I2C address to final one */
				status = VL53L0X_SetDeviceAddress(&Dev[i], FinalAddress);
				HAL_Delay(2);
				if (status != 0) {
					printf("%i VL53L0X_SetDeviceAddress fail\n", i);
					break;
				}
				Dev[i].I2cDevAddr = FinalAddress;
				/* Check all is OK with the new I2C address and initialize the sensor */
				status = VL53L0X_RdWord(&Dev[i],
						VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
				HAL_Delay(2);
				if (status != 0) {
					printf("%i VL53L0X_RdWord fail\n", i);
					break;
				}
				status = VL53L0X_DataInit(&Dev[i]);
				HAL_Delay(2);
				if (status == 0) {
					Dev[i].Present = 1;
				} else {
					printf("VL53L0X_DataInit %d fail\n", i);
					break;
				}
				Dev[i].Present = 1;
			} else {
				printf("#%d unknown ID %x\n", i, Id);
				status = 1;
			}
		} while (0);
		HAL_Delay(10);
		VL53L0X_InitContinuous(i);
		HAL_Delay(2);
	}
	HAL_Delay(10);
	VL53L0X_EnableInterrupts();
	HAL_Delay(2);
	printf(
			!status ?
					"VL53L0X All Chips booted correctly\n" :
					"VL53L0X Booting error\n");
	//vls_initialized = TRUE;
}

void VL53L0X_ResetAllSensors() {
	HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(XSHUT2_GPIO_Port, XSHUT2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(XSHUT3_GPIO_Port, XSHUT3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(XSHUT4_GPIO_Port, XSHUT4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(XSHUT5_GPIO_Port, XSHUT5_Pin, GPIO_PIN_RESET);
}

void VL53L0X_SensorTurnOn(int sens) {
	switch (sens) {
	case 0:
		HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(XSHUT2_GPIO_Port, XSHUT2_Pin, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(XSHUT3_GPIO_Port, XSHUT3_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(XSHUT4_GPIO_Port, XSHUT4_Pin, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(XSHUT5_GPIO_Port, XSHUT5_Pin, GPIO_PIN_SET);
		break;
	}
}

void VL53L0X_DisableInterrupts() {
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void VL53L0X_EnableInterrupts() {

	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void VL53L0X_InitContinuous(int s) {
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	VL53L0X_WaitDeviceBooted(&Dev[s]);
	VL53L0X_DataInit(&Dev[s]);
	VL53L0X_StaticInit(&Dev[s]);
	VL53L0X_PerformRefCalibration(&Dev[s], &VhvSettings, &PhaseCal);
	VL53L0X_PerformRefSpadManagement(&Dev[s], &refSpadCount, &isApertureSpads);
	VL53L0X_SetDeviceMode(&Dev[s], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	VL53L0X_StartMeasurement(&Dev[s]);
}

uint16_t VL53L0X_ContinuousRequest(int s) {
	if (TOF_DATA_READ[s] == TRUE) {
		TOF_DATA_READ[s] = FALSE;
		return RangingData[s].RangeMilliMeter;
	} else
		return -1;
}

void VL53L0X_PerformMeasurement(int sensor) {
	VL53L0X_GetRangingMeasurementData(&Dev[sensor], &RangingData[sensor]);
	VL53L0X_ClearInterruptMask(&Dev[sensor],
			VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	DIST[sensor] = RangingData[sensor].RangeMilliMeter;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case INT1_Pin:
		TOF_DATA_READ[0] = TRUE;
		VL53L0X_PerformMeasurement(0);
		// ledToggle(1);
		break;
	case INT2_Pin:
		TOF_DATA_READ[1] = TRUE;
		VL53L0X_PerformMeasurement(1);
		// ledToggle(2);
		break;
	case INT3_Pin:
		TOF_DATA_READ[2] = TRUE;
		VL53L0X_PerformMeasurement(2);
		//  ledToggle(3);
		break;
	case INT4_Pin:
		TOF_DATA_READ[3] = TRUE;
		VL53L0X_PerformMeasurement(3);
		//  ledToggle(4);
		break;
	case INT5_Pin:
		TOF_DATA_READ[4] = TRUE;
		VL53L0X_PerformMeasurement(4);
		//  ledToggle(5);
		break;
	case MPUINT_Pin:
		MPUInterruptHandler();
		break;
	}
}

//void TaskManagerInit(uint32_t task_list){
//    //default
//    start_task_list[BATTERY_LEVEL_ID] = TRUE;
//    start_task_list[BUTTONS_CONTROL_ID] = TRUE;
//    start_task_list[DISTANCE_SENSORS_ID] = FALSE;
//    start_task_list[IMU_SENSORS_ID] = FALSE;
//    start_task_list[FLOOD_FILL_ID] = FALSE;
//    start_task_list[BELLMAN_FORD_ID] = FALSE;
//    start_task_list[DIJKSTRA_SEARCH_ID] = FALSE;
//    start_task_list[KALMAN_FILTER_ID] = FALSE;
//    start_task_list[LANDMARK_DETECTION_ID] = FALSE;
//    start_task_list[EEPROM_MEMORY_ID] = FALSE;
//    start_task_list[PID_CONTROL_ID] = TRUE;
//    start_task_list[UART_RECEIVE_ID] = TRUE;
//    start_task_list[UART_TRANSMIT_ID] = TRUE;
//    start_task_list[PATH_PLANNER_ID] = FALSE;
//    //DEFAULT 01110000000101
//
//    start_task_list[BATTERY_LEVEL_ID] = (task_list&(1<<BATTERY_LEVEL_ID))>0;
//    start_task_list[BUTTONS_CONTROL_ID] = (task_list&(1<<BUTTONS_CONTROL_ID))>0;
//    start_task_list[DISTANCE_SENSORS_ID] = (task_list&(1<<DISTANCE_SENSORS_ID))>0;
//    start_task_list[IMU_SENSORS_ID] = (task_list&(1<<IMU_SENSORS_ID))>0;
//    start_task_list[FLOOD_FILL_ID] = (task_list&(1<<FLOOD_FILL_ID))>0;
//    start_task_list[BELLMAN_FORD_ID] = (task_list&(1<<BELLMAN_FORD_ID))>0;
//    start_task_list[DIJKSTRA_SEARCH_ID] = (task_list&(1<<DIJKSTRA_SEARCH_ID))>0;
//    start_task_list[KALMAN_FILTER_ID] = (task_list&(1<<KALMAN_FILTER_ID))>0;
//    start_task_list[LANDMARK_DETECTION_ID] = (task_list&(1<<LANDMARK_DETECTION_ID))>0;
//    start_task_list[EEPROM_MEMORY_ID] = (task_list&(1<<EEPROM_MEMORY_ID))>0;
//    start_task_list[PID_CONTROL_ID] = (task_list&(1<<PID_CONTROL_ID))>0;
//    start_task_list[UART_RECEIVE_ID] = (task_list&(1<<UART_RECEIVE_ID))>0;
//    start_task_list[UART_TRANSMIT_ID] = (task_list&(1<<UART_TRANSMIT_ID))>0;
//    start_task_list[PATH_PLANNER_ID] = (task_list&(1<<PATH_PLANNER_ID))>0;
//}

void battery_task() {
	battery_voltage = (float) ((raw_voltage_adc) / 490.196f);
	if (raw_voltage_adc < BATTERY_LOW_VOLTAGE_THRESHOLD) {
		ledEnable(PWR_LED);
	}
	if (raw_voltage_adc < BATTERY_CRITICAL_VOLTAGE_THRESHOLD) {
		performLowVoltageAction();
		while (1) {
			ledToggle(5);
			HAL_Delay(1000);
			ledToggle(5);
			HAL_Delay(50);
		}
	}
	runBatteryLevelADC(&raw_voltage_adc);
	if(bat_debug_flag)
		printf("%d.%02d V\n",(int)battery_voltage, (int)(battery_voltage*100) % 100);
	else
		__NOP();
}

void lowlvl_init(void) {
	// licznik uruchomien
	startup_counter_data = eepromRead4Bytes(startup_counter_addr);
	startup_counter_data++;
	eepromSave4Bytes(&startup_counter_addr, &startup_counter_data);

	// load from EXT MEM other important variables
	vP = (float)(int)eepromRead4Bytes(vp_addr);
	vI = (float)(int)eepromRead4Bytes(vi_addr);
	vD = (float)(int)eepromRead4Bytes(vd_addr);
	wP = (float)(int)eepromRead4Bytes(wp_addr);
	wI = (float)(int)eepromRead4Bytes(wi_addr);
	wD = (float)(int)eepromRead4Bytes(wd_addr);
//	EEPROMMemoryInit();
//	 TaskManagerInit(eepromRead4Bytes(0));
//	    if(start_task_list[BATTERY_LEVEL_ID])BatteryLevelInit();
	raw_voltage_adc = BATTERY_LEVEL_ADC_INITIAL_VALUE;
	runBatteryLevelADC(&raw_voltage_adc);
	ledOutputInit();
//	    // bardzo ważne timery, generowanie PWMa dla ledów, wyzwalanie w INT MPU, tim7 zliczanie millis()
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start_IT(&htim4);
//	    if(start_task_list[BUTTONS_CONTROL_ID])ButtonsControlInit();
	int welcome_blinks = 6;
	while (welcome_blinks-- > 0) {
		ledToggle(1);
		ledToggle(2);
		ledToggle(3);
		ledToggle(4);
		HAL_Delay(200);
	}
//	    if(start_task_list[UART_RECEIVE_ID]||start_task_list[UART_TRANSMIT_ID])UARTCommunicationInit();
	UARTDMA_Init(&huartdma, &huart3);
	printf("MICROMOUSE VRISKER by KN INTEGRA\nStartup number %d\n\n", (int)startup_counter_data);
//	    if(start_task_list[DISTANCE_SENSORS_ID])DistanceSensorsInit();
	//VL53L0X_Init();
//	    if(start_task_list[IMU_SENSORS_ID])IMUSensorsInit();
	//IMU_MPU6050_Init();
	HAL_Delay(100);
//	    if(start_task_list[PID_CONTROL_ID])PIDControlInit();
	encodersInit();
	motorsPWMInit();
	HAL_Delay(100);
	HAL_TIM_Base_Start_IT(&htim9);
//	    if(start_task_list[KALMAN_FILTER_ID])KalmanFilterInit();
//	    if(start_task_list[EEPROM_MEMORY_ID])EEPROMMemoryInit();
//	    if(start_task_list[BELLMAN_FORD_ID])BellmanFordInit();
//	    if(start_task_list[DIJKSTRA_SEARCH_ID])DijkstraSearchInit();
//	    if(start_task_list[FLOOD_FILL_ID])FloodFillInit();
//	    if(start_task_list[LANDMARK_DETECTION_ID])LandmarkDetectionInit();
//	//    if(start_task_list[PATH_PLANNER_ID])PathPlannerInit();
	//HAL_TIM_Base_Start_IT(&htim7);
	HAL_Delay(10);
	HAL_TIM_Base_Start_IT(&htim10);
	ledEnable(1);
}

