#ifndef FAKE_EEPROM_H
#define FAKE_EEPROM_H

#include <stdint.h>

#include "main.h"

HAL_StatusTypeDef clear_eeprom();
HAL_StatusTypeDef read_eeprom(uint32_t addr, uint16_t data_size, uint8_t data[]);
HAL_StatusTypeDef write_eeprom(uint32_t addr, uint16_t data_size, uint8_t data[]);

#endif // FAKE_EEPROM_H