//*****************************************************************************
//
// File Name	: 'flash.c'
// Author	: Steve NGUYEN
// Contact      : steve.nguyen@pollen-robotics.com
// Created	: Tuesday, February 22 2022
// Revised	:
// Version	:
// Target MCU	:
//
//*****************************************************************************


#include "flash.h"
#include <string.h>
#include "main.h"


void LuosHAL_FlashEraseLuosMemoryInfo(void)
{
    uint32_t page_error = 0;
    FLASH_EraseInitTypeDef s_eraseinit;

    s_eraseinit.TypeErase = FLASH_TYPEERASE_PAGES;
    s_eraseinit.Page      = NB_OF_PAGE - 1;
    s_eraseinit.NbPages   = 1;

    // Erase Page
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&s_eraseinit, &page_error);
    HAL_FLASH_Lock();
    for(int i=0;i<5;i++)
    {
      status_led(1);
      HAL_Delay(100);
      status_led(0);
      HAL_Delay(100);
    }

}
/******************************************************************************
 * @brief Write flash page where Luos keep permanente information
 * @param Address page / size to write / pointer to data to write
 * @return
 ******************************************************************************/
void LuosHAL_FlashWriteLuosMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data)
{
    // Before writing we have to erase the entire page
    // to do that we have to backup current falues by copying it into RAM
    uint8_t page_backup[PAGE_SIZE];
    memcpy(page_backup, (void *)ADDRESS_ALIASES_FLASH, PAGE_SIZE);

    // Now we can erase the page
    LuosHAL_FlashEraseLuosMemoryInfo();

    // Then add input data into backuped value on RAM
    uint32_t RAMaddr = (addr - ADDRESS_ALIASES_FLASH);
    memcpy(&page_backup[RAMaddr], data, size);

    // and copy it into flash
    HAL_FLASH_Unlock();

    // ST hal flash program function write data by uint64_t raw data
    for (uint32_t i = 0; i < PAGE_SIZE; i += sizeof(uint64_t))
    {
        while (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, i + ADDRESS_ALIASES_FLASH, *(uint64_t *)(&page_backup[i])) != HAL_OK)
            ;
    }
    HAL_FLASH_Lock();
    for(int i=0;i<5;i++)
    {
      status_led(1);
      HAL_Delay(100);
      status_led(0);
      HAL_Delay(100);
    }

}
/******************************************************************************
 * @brief read information from page where Luos keep permanente information
 * @param Address info / size to read / pointer callback data to read
 * @return
 ******************************************************************************/
void LuosHAL_FlashReadLuosMemoryInfo(uint32_t addr, uint16_t size, uint8_t *data)
{
    memcpy(data, (void *)(addr), size);
}
