/*
 * lowlvl.h
 *
 *  Created on: 26 paź 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_LOWLVL_H_
#define INC_LOWLVL_H_
#include "main.h"
#include "vl53l0x_api.h"
#include <mpu6050.h>
#include <PWM_access.h>
#include "drive_driver.h"

#define BATTERY_LOW_VOLTAGE_THRESHOLD 3480
#define BATTERY_CRITICAL_VOLTAGE_THRESHOLD 3000
#define BATTERY_LEVEL_ADC_INITIAL_VALUE ((1<<12)-1)
#define BATLVL_TASK_DELAY 1000
#define PWR_LED 5

#define DISTANCE_BETWEEN_WHEELS (76)
#define ENCODER_TICKS_PER_FULL_WHEEL_ROTATION (616)  // przełożenie silnika 51.45:1 * 12 impulsów na obrót enkodera

#define I2C_TIMEOUT 100
#define DELAY_IMU_CYCLE_WRITE 1
#define DELAY_IMU_CYCLE_READ 2

#define LED1_OUTPUT_PWM 10
#define LED2_OUTPUT_PWM 10
#define LED3_OUTPUT_PWM 10
#define LED4_OUTPUT_PWM 10
#define LED5_OUTPUT_PWM 50

#define DELAY_EEPROM_CYCLE_WRITE 2
#define DELAY_EEPROM_CYCLE_READ 2
#define EEPROM_I2C_TIMEOUT 20
#define EEPROM_DEV_ADDRESS 0xa0

#define DMA_RX_BUFFER_SIZE 64
#define UART_RECEIVING_BUFFER_SIZE 256
#define DMA_TX_BUFFER_SIZE 128
#define UART_SENDING_BUFFER_SIZE 512

extern int16_t receivedMPUData[7];
extern uint16_t DIST[5];
extern uint16_t vp_addr;
extern uint32_t vp_data;
extern uint16_t vi_addr;
extern uint32_t vi_data;
extern uint16_t vd_addr;
extern uint32_t vd_data;
extern uint16_t wp_addr;
extern uint32_t wp_data;
extern uint16_t wi_addr;
extern uint32_t wi_data;
extern uint16_t wd_addr;
extern uint32_t wd_data;

typedef struct
{
    UART_HandleTypeDef* huart; // UART handler

    uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE]; // DMA direct buffer
    uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE]; // DMA direct buffer
    uint8_t UART_RX_Buffer[UART_RECEIVING_BUFFER_SIZE]; // UART working circular buffer


    uint16_t UartBufferHead;
    uint16_t UartBufferTail;
    uint8_t UartBufferLines;
}UARTDMA_HandleTypeDef;

void eepromMemoryDump(uint32_t timeout);
void eepromFullMemoryErase();
uint32_t eepromRead4Bytes(uint16_t addr);
void eepromSave4Bytes(uint16_t *addr, uint32_t *data);
void eepromMemoryTestRW();

void eepromMemoryTestErase();

void runBatteryLevelADC(uint32_t *variable);
void performLowVoltageAction();

GPIO_PinState BTN_IsPressed(uint8_t btnid);

void encodersInit();
int32_t getLeftTickCount();
int32_t getRightTickCount();
void setEncoderCount(int id, int16_t value);
void resetLeftTickCount();
void resetRightTickCount();

void ledOutputInit();
void ledToggle(uint16_t ledx);
void ledEnable(uint16_t ledx);
void ledDisable(uint16_t ledx);
void generateLEDPWMCallback(void);

void motorsPWMInit();
void motorLeftSetPWM(int32_t duty);
void motorRightSetPWM(int32_t duty);
int motorLeftGetPWM();
int motorRightGetPWM();

void IMU_MPU6050_Init(void);
int getMPUDataReadyStatus();
void setMPUDataReadyStatus(int enable);
void MPU6050_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
void IMU_MPU6050_ReadRaw(int16_t *buff);
void MPUInterruptHandler(void);

void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart);
void UARTDMA_UartIrqHandler(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_DmaIrqHandler(UARTDMA_HandleTypeDef *huartdma);
uint8_t UARTDMA_IsDataReady(UARTDMA_HandleTypeDef *huartdma);
int UARTDMA_GetLineFromBuffer(UARTDMA_HandleTypeDef *huartdma, char *OutBuffer);
void uart_rx(void);
void uart_tx(void);

void VL53L0X_Init();
uint16_t VL53L0X_ContinuousRequest(int);
void VL53L0X_ResetAllSensors();
void VL53L0X_SensorTurnOn(int);
void VL53L0X_InitContinuous(int);
void VL53L0X_DisableInterrupts();
void VL53L0X_EnableInterrupts();
void VL53L0X_PerformMeasurement(int sensor);
VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice);
void battery_task();

void lowlvl_init(void);

#endif /* INC_LOWLVL_H_ */
