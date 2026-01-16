/*
 * ICM42688P.c
 *
 *  Created on: Oct 8, 2025
 *      Author: leeli
 */

#include "ICM42688P.h"
extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;
ICM42688P_Data_t imudata;
uint8_t buf[15];


uint8_t hal_Spi1_ReadWriteByte(uint8_t txdata)
{
    uint8_t rxdata = 0;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rxdata, 1, 5); /* STM32 HAL SPI transmit/receive function */
    return rxdata;
}

void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
{
    uint8_t i = 0;
    for(i = 0; i < len; i++)
    {
        *pBuffer = hal_Spi1_ReadWriteByte(*pBuffer);
        pBuffer++;
    }
}


uint8_t ICM42688P_ReadReg(uint8_t reg)
{
    uint8_t regval = 0;
    Icm_CS_LOW();                /* Pull CS low to start SPI transaction */
    reg |= 0x80;                 /* Set MSB (D7) = 1 → SPI read operation */
    Icm_Spi_ReadWriteNbytes(&reg, 1);   /* Send register address */
    Icm_Spi_ReadWriteNbytes(&regval, 1);/* Receive register data */
    Icm_CS_HIGH();               /* Pull CS high to end SPI transaction */
    return regval;
}


void ICM42688P_ReadRegs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    reg |= 0x80;
    Icm_CS_LOW();
    Icm_Spi_ReadWriteNbytes(&reg, 1);   /* Send starting register address */
    Icm_Spi_ReadWriteNbytes(buf, len);  /* Read multiple bytes into buffer */
    Icm_CS_HIGH();
}

uint8_t ICM42688P_WriteReg(uint8_t reg, uint8_t value)
{
    Icm_CS_LOW();
    Icm_Spi_ReadWriteNbytes(&reg, 1);   /* Send register address */
    Icm_Spi_ReadWriteNbytes(&value, 1); /* Write data to the register */
    Icm_CS_HIGH();
    return 0;
}

/**
 * @brief  Initialize the ICM-42688-P IMU.
 *
 * This function performs a full initialization sequence for the ICM-42688-P,
 * including soft reset, communication check, and configuration of accelerometer
 * and gyroscope parameters. It ensures the sensor starts in a known and stable
 * state before use.
 */
void ICM42688P_Init(void)
{
	/* (1) wait 50ms for the sensor to power up completely */
    HAL_Delay(50);
    /* (2) soft reset */
    // select Bank 0
    ICM42688P_WriteReg(MPUREG_REG_BANK_SEL, 0);
    // soft reset
    uint8_t data = ICM426XX_DEVICE_CONFIG_RESET_EN;
    ICM42688P_WriteReg(MPUREG_DEVICE_CONFIG, data);
    HAL_Delay(50);
    /* (3) check Device ID*/
    uint8_t who_am_i = ICM42688P_ReadReg(MPUREG_WHO_AM_I);
    if (who_am_i != ICM_WHOAMI) {
        while(1){
      	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
          	 HAL_Delay(10);
        };
    }
    /* (4)Configure Low noise mode*/
    // select Bank 0
    ICM42688P_WriteReg(MPUREG_REG_BANK_SEL, 0);
    // set LN mode for gyro and accel
    ICM42688P_WriteReg(MPUREG_PWR_MGMT_0, ICM426XX_PWR_MGMT_0_GYRO_MODE_LN | ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN);
    /* (5) Configure FSR and ODR*/
    // Gyroscope Configuration: Set 500dps FSR and 1kHz ODR.
    ICM42688P_WriteReg(MPUREG_GYRO_CONFIG0, ICM426XX_GYRO_CONFIG0_FS_SEL_500dps | ICM426XX_GYRO_CONFIG0_ODR_1_KHZ);
    // Accelerometer Configuration: Set 4g FSR and 1kHz ODR.
    ICM42688P_WriteReg(MPUREG_ACCEL_CONFIG0, ICM426XX_ACCEL_CONFIG0_FS_SEL_4g | ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ);
    /* (6) Disable FIFO stream*/
    // select Bank 0
    ICM42688P_WriteReg(MPUREG_REG_BANK_SEL, 0);
    data = ICM426XX_FIFO_CONFIG_MODE_BYPASS;
    ICM42688P_WriteReg(MPUREG_FIFO_CONFIG, data);
    /* (7) Configure the interrupt mode*/
    // -set pulse mode -- the INT1 pin generates a short, fixed-duration pulse upon an interrupt event.
    // The pulse automatically self-clears after its duration.
    // -set Drive circuit to push pull-Since IMU board lacks the external pull-up resistor for the int1 pin,
    // the Push-Pull configuration is needed to ensure the interrupt signal can successfully transition to
    // the HIGH state when the interrupt is inactive or cleared.
    //- set polarity high- The INT1 pin is driven High when an interrupt occurs.
    ICM42688P_WriteReg(MPUREG_INT_CONFIG,ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT_CONFIG_INT1_POLARITY_HIGH);
    // INT_SOURCE0--we care about two events to trigger interrupt signal:
    // imu reset and imu data ready
    ICM42688P_WriteReg(MPUREG_INT_SOURCE0, BIT_INT_SOURCE0_RESET_DONE_INT1_EN |BIT_INT_SOURCE0_UI_DRDY_INT1_EN);
    HAL_Delay(50);
}

float ICM42688P_ConvertAccel(int32_t raw_accel, float sensitivity) {
    return (float)raw_accel * sensitivity;
}

float ICM42688P_ConvertGyro(int32_t raw_gyro, float sensitivity) {
    return (float)raw_gyro * sensitivity;
}

float ICM42688P_ConvertTemp(int16_t raw_temp) {
    return ((float)raw_temp / ICM42688P_TEMPERATURE_SENSITIVITY) + ICM42688P_TEMPERATURE_OFFSET;
}

uint8_t ICM42688P_read_dma(void)
{
	uint8_t tx_buf[15];
	//uint8_t buf[15]; // 1 byte Dummy + 14 bytes data
    tx_buf[0] = 0x1D | 0x80; // Set MSB=1 to indicate read
    Icm_CS_LOW();
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, buf, 15);
    return 1;
}
uint16_t last_count = 0;
uint8_t ICM42688P_decode(ICM42688P_Data_t *data){

	data->temperature_C = ICM42688P_ConvertTemp((int16_t)((buf[1] << 8) | buf[2]));

	data->accel_x = ICM42688P_ConvertAccel((int16_t)((buf[3] << 8) | buf[4]),ICM42688P_ACCEL_4G_SENSITIVITY);
	data->accel_y = ICM42688P_ConvertAccel((int16_t)((buf[5] << 8) | buf[6]),ICM42688P_ACCEL_4G_SENSITIVITY);
	data->accel_z = ICM42688P_ConvertAccel((int16_t)((buf[7] << 8) | buf[8]),ICM42688P_ACCEL_4G_SENSITIVITY);
    data->gyro_x = ICM42688P_ConvertGyro((int16_t)((buf[9] << 8) | buf[10]),ICM42688P_GYRO_500DPS_SENSITIVITY);
    data->gyro_y = ICM42688P_ConvertGyro((int16_t)((buf[11] << 8) | buf[12]),ICM42688P_GYRO_500DPS_SENSITIVITY);
    data->gyro_z = ICM42688P_ConvertGyro((int16_t)((buf[13] << 8) | buf[14]),ICM42688P_GYRO_500DPS_SENSITIVITY);

    // 1. Read current counter value immediately
    uint16_t current_count = __HAL_TIM_GET_COUNTER(&htim1);
    uint16_t delta_us = 0;
    // 2. Compute difference (handle 16-bit counter overflow/wraparound)
    if (current_count >= last_count) {
    	delta_us = current_count - last_count;
    } else {
        // Overflow/wraparound occurred: (max - last) + current + 1
        delta_us = (0xFFFF - last_count) + current_count + 1;
    }


    // 3. Update last value for next use
    last_count = current_count;

    data->dt = delta_us;
    return 1;
}

uint8_t ICM42688P_GetData(ICM42688P_Data_t *data) {
	ICM42688P_read_dma();

	ICM42688P_decode(&imudata);
    return 1;
}

