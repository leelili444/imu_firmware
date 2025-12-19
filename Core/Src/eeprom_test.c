/**
 ******************************************************************************
 * @file    eeprom_test.c
 * @brief   AT24C256 EEPROM test and driver example for STM32F405
 * @description
 *   This file demonstrates how to initialize I2C, write data to an external
 *   EEPROM (AT24C256), and read it back for verification using STM32 HAL.
 ******************************************************************************
 */

#include "main.h"
#include "eeprom_test.h"
#include <string.h>
#include <stdio.h>



extern I2C_HandleTypeDef hi2c1;                     // imported from main.c

/* -------------------------------------------------------------------------- */
/*                          EEPROM driver functions                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Write a block of data to the EEPROM
 * @param  MemAddress: 16-bit memory address in EEPROM
 * @param  pData: pointer to data buffer
 * @param  Size: number of bytes to write
 * @retval HAL status
 */
HAL_StatusTypeDef EEPROM_Write(uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
    /* Note:
       AT24C256 uses 16-bit memory addressing.
       Ensure write size does not cross a 64-byte page boundary.
    */
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&EEPROM_I2C,
                               EEPROM_ADDRESS,
                               MemAddress,
                               I2C_MEMADD_SIZE_16BIT,
                               pData,
                               Size,
                               100);

    /* Wait for internal write cycle */
    HAL_Delay(EEPROM_WRITE_DELAY_MS);
    return status;
}

/**
 * @brief  Read a block of data from the EEPROM
 * @param  MemAddress: 16-bit memory address in EEPROM
 * @param  pData: pointer to buffer to store read data
 * @param  Size: number of bytes to read
 * @retval HAL status
 */
HAL_StatusTypeDef EEPROM_Read(uint16_t MemAddress, uint8_t *pData, uint16_t Size)
{
    return HAL_I2C_Mem_Read(&EEPROM_I2C,
                            EEPROM_ADDRESS,
                            MemAddress,
                            I2C_MEMADD_SIZE_16BIT,
                            pData,
                            Size,
                            100);
}

/* -------------------------------------------------------------------------- */
/*                              Test function                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief  EEPROM functional test
 * @note   Write a test string to address 0x0000 and read it back.
 */
uint8_t writeData[] = "Hello EEPROM!";
uint8_t readData[32] = {0};

void EEPROM_Test(void)
{
    /* -------------------- Write test -------------------- */
    printf("EEPROM Write Start...\r\n");
    if (EEPROM_Write(0x0000, writeData, strlen((char*)writeData)) == HAL_OK)
        printf("EEPROM Write Done.\r\n");
    else
        printf("EEPROM Write Error!\r\n");

    /* -------------------- Read test --------------------- */
    printf("EEPROM Read Start...\r\n");
    if (EEPROM_Read(0x0000, readData, strlen((char*)writeData)) == HAL_OK)
    {
        printf("EEPROM Read Done.\r\n");
        printf("Read Data: %s\r\n", readData);
    }
    else
    {
        printf("EEPROM Read Error!\r\n");
    }
}

/*  Key notes
- AT24C256 uses 16-bit addressing.
- Each page = 64 bytes, do not cross page boundary in one write.
- EEPROM_ADDRESS may vary depending on hardware wiring of A0~A2.
- Always include a delay (~10 ms) after writing.
- Confirm I2C lines have proper pull-ups and no bus contention.
*/
