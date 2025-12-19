/*
 * ICM42688P.h
 *
 *  Created on: Oct 8, 2025
 *      Author: leeli
 */

#ifndef INC_ICM42688P_H_
#define INC_ICM42688P_H_

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "Icm426xxDefs.h"

// Sensor Data Structure
//
typedef struct {
    uint32_t timestamp;
    uint32_t dt;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature_C;
} ICM42688P_Data_t;


#define ICM42688P_TEMPERATURE_SENSITIVITY 132.48f
#define ICM42688P_TEMPERATURE_OFFSET      25.0f

// FSR sensitivity values
#define ICM42688P_ACCEL_16G_SENSITIVITY   (1.0f / 2048.0f) // LSB per g for 16G FSR (16G量程下，每g的LSB值)
#define ICM42688P_ACCEL_8G_SENSITIVITY    (1.0f / 4096.0f)
#define ICM42688P_ACCEL_4G_SENSITIVITY    (1.0f / 8192.0f)
#define ICM42688P_ACCEL_2G_SENSITIVITY    (1.0f / 16384.0f)
#define ICM42688P_GYRO_2000DPS_SENSITIVITY (1.0f / 16.4f)  // LSB per dps for 2000dps FSR
#define ICM42688P_GYRO_1000DPS_SENSITIVITY (1.0f / 32.8f)
#define ICM42688P_GYRO_500DPS_SENSITIVITY  (1.0f / 65.5f)
#define ICM42688P_GYRO_250DPS_SENSITIVITY  (1.0f / 131.0f)
#define ICM42688P_GYRO_125DPS_SENSITIVITY  (1.0f / 262.0f)




#define Icm_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define Icm_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)

uint8_t hal_Spi1_ReadWriteByte(uint8_t txdata);

void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len);

uint8_t ICM42688P_ReadReg(uint8_t reg);
void ICM42688P_ReadRegs(uint8_t reg, uint8_t* buf, uint16_t len);

uint8_t ICM42688P_WriteReg(uint8_t reg, uint8_t value);

void ICM42688P_Init();

float ICM42688P_ConvertTemp(int16_t raw_temp);
float ICM42688P_ConvertAccel(int32_t raw_accel, float sensitivity);
float ICM42688P_ConvertGyro(int32_t raw_gyro, float sensitivity);
uint8_t ICM42688P_read_dma(void);
uint8_t ICM42688P_decode(ICM42688P_Data_t *data);
uint8_t ICM42688P_GetData(ICM42688P_Data_t *data);

#endif /* INC_ICM42688P_H_ */
