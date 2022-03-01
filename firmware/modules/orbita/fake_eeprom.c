#include "fake_eeprom.h"

#include <string.h>

#include "main.h"
#include "stm32g4xx_hal_flash.h"

#include "utils.h"

#define ADDR_FLASH_BANK2 ((uint32_t)0x08040000)

uint32_t get_eeprom_address()
{
    return ADDR_FLASH_BANK2 + (uint32_t)((FLASH_PAGE_NB - 1) * FLASH_PAGE_SIZE);
}

HAL_StatusTypeDef flashEraseMemoryInfo(void)
{
    uint32_t page_error = 0;
    FLASH_EraseInitTypeDef s_eraseinit;

    s_eraseinit.TypeErase = FLASH_TYPEERASE_PAGES;
    s_eraseinit.Banks = FLASH_BANK_2;
    s_eraseinit.Page = FLASH_PAGE_NB - 1;
    s_eraseinit.NbPages = 1;

    // Erase Page
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    HAL_StatusTypeDef ret = HAL_FLASHEx_Erase(&s_eraseinit, &page_error);

    HAL_FLASH_Lock();

    return ret;
}

HAL_StatusTypeDef flashWriteMemoryInfo(uint32_t addr, uint16_t data_size, uint8_t data[])
{
    uint32_t eeprom_base_addr = get_eeprom_address();

    uint8_t page_backup[FLASH_PAGE_SIZE];
    memcpy(page_backup, (void *)eeprom_base_addr, FLASH_PAGE_SIZE);

    // Now we can erase the page
    HAL_StatusTypeDef ret = flashEraseMemoryInfo();
    if (ret != HAL_OK)
    {
        return ret;
    }

    // Then add input data into backuped value on RAM
    memcpy(page_backup + addr, data, data_size);

    // and copy it into flash
    HAL_FLASH_Unlock();

    // ST hal flash program function write data by uint64_t raw data
    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i += sizeof(uint64_t))
    {
        HAL_StatusTypeDef ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, i + eeprom_base_addr, *(uint64_t *)(&page_backup[i]));

        if (ret != HAL_OK)
        {
            return ret;

        }
    }

    HAL_FLASH_Lock();

    return HAL_OK;
}

void flashReadMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data)
{
    memcpy(data, (void *)(get_eeprom_address() + addr), size);
}


#define DUMMY_SET_VALUE 42

HAL_StatusTypeDef clear_eeprom()
{
    return flashEraseMemoryInfo();
}

HAL_StatusTypeDef read_eeprom(uint32_t addr, uint16_t size, uint8_t *data)
{
    uint8_t is_set;
    flashReadMemoryInfo(addr, sizeof(uint8_t), &is_set);

    if (is_set == DUMMY_SET_VALUE)
    {
        flashReadMemoryInfo(addr + sizeof(uint8_t), size, data);
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}


HAL_StatusTypeDef write_eeprom(uint32_t addr, uint16_t size, uint8_t *data)
{
    uint8_t tmp[size + sizeof(uint8_t)];
    tmp[0] = DUMMY_SET_VALUE;
    memcpy(tmp + sizeof(uint8_t), data, size);

    return flashWriteMemoryInfo(addr, size + sizeof(uint8_t), tmp);
}