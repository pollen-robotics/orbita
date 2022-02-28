#ifndef FAKE_EEPROM_H
#define FAKE_EEPROM_H

#include <stdint.h>

void read_eeprom(uint32_t addr, uint16_t size, uint8_t *data);
void write_eeprom(uint32_t addr, uint16_t size, uint8_t *data);

#endif // FAKE_EEPROM_H