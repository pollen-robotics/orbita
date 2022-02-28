#include "fake_eeprom.h"

#include <string.h>

#include "flash.h"
#include "utils.h"

#define DUMMY_SET_VALUE 42


void read_eeprom(uint32_t addr, uint16_t size, uint8_t *data)
{
    uint8_t is_set;
    flashReadMemoryInfo(addr, 1, &is_set);

    if (is_set == DUMMY_SET_VALUE)
    {
        flashReadMemoryInfo(addr + 1, size, data);
    }
}


void write_eeprom(uint32_t addr, uint16_t size, uint8_t *data)
{
    // toggle_status_led();

    // uint8_t tmp[size + 1];
    // tmp[0] = DUMMY_SET_VALUE;
    // memcpy(tmp + 1, data, size);

    flashWriteMemoryInfo(addr, size, data);
}