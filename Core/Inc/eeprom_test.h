
#ifndef INC_EEPROM_TEST_H_
#define INC_EEPROM_TEST_H_

/* -------------------------------------------------------------------------- */
/*                          EEPROM I2C configuration                          */
/* -------------------------------------------------------------------------- */
#define EEPROM_I2C                hi2c1              // I2C handle (defined in main.c)
#define EEPROM_ADDRESS            0xA0              // AT24C256 address (A2–A0 = GND)
#define EEPROM_PAGE_SIZE          64                // 64 bytes per page
#define EEPROM_WRITE_DELAY_MS     10                // typical write cycle time

void EEPROM_Test(void);
HAL_StatusTypeDef EEPROM_Read(uint16_t MemAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef EEPROM_Write(uint16_t MemAddress, uint8_t *pData, uint16_t Size);

#endif /* INC_EEPROM_TEST_H_ */
