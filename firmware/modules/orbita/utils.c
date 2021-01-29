#include "utils.h"
#include "luos_hal.h"


void Orbita_FlashEraseLuosMemoryInfo(void)
{
    uint32_t page_error = 0;
    FLASH_EraseInitTypeDef s_eraseinit;

    s_eraseinit.TypeErase = FLASH_TYPEERASE_PAGES;
    s_eraseinit.Banks = FLASH_BANK_2;
    s_eraseinit.Page = FLASH_PAGE_NB - 1;
    s_eraseinit.NbPages = 1;

    // Erase Page
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
    HAL_FLASH_Lock();
}

void Orbita_FlashWriteLuosMemoryInfo(uint32_t addr, uint16_t data_size, uint8_t data[])
{
    uint32_t aliases_base_addr = ADDR_FLASH_BANK2 + (uint32_t)((FLASH_PAGE_NB - 1) * PAGE_SIZE);

    addr += aliases_base_addr;

    uint8_t page_backup[PAGE_SIZE];
    memcpy(page_backup, (void *)aliases_base_addr, PAGE_SIZE);

    // Now we can erase the page
    Orbita_FlashEraseLuosMemoryInfo();

    // Then add input data into backuped value on RAM
    uint32_t RAMaddr = (addr - aliases_base_addr);
    memcpy(&page_backup[RAMaddr], data, data_size);

    // and copy it into flash
    HAL_FLASH_Unlock();

    // ST hal flash program function write data by uint64_t raw data
    for (uint32_t i = 0; i < PAGE_SIZE; i += sizeof(uint64_t))
    {
        while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, i + aliases_base_addr, *(uint64_t *)(&page_backup[i])) != HAL_OK)
            ;
    }
    HAL_FLASH_Lock();
}

void Orbita_FlashReadLuosMemoryInfo(uint32_t addr, uint16_t data_size, uint8_t *data)
{
    uint32_t aliases_base_addr = ADDR_FLASH_BANK2 + (uint32_t)((FLASH_PAGE_NB - 1) * PAGE_SIZE);

    addr += aliases_base_addr;

    memcpy(data, (void *)addr, data_size);
}