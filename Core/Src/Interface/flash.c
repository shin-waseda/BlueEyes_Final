#include "global.h"
#include "interface.h"

HAL_StatusTypeDef flash_lock() { return HAL_FLASH_Lock(); }

HAL_StatusTypeDef flash_unlock_erase() {
  HAL_StatusTypeDef status;

  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = EEPROM_START_ADDRESS;
  EraseInitStruct.NbPages = 1;

  status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
    return status;
  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
  return status;
}

HAL_StatusTypeDef flash_write_2byte(uint32_t address, uint16_t data) {
  HAL_StatusTypeDef status;
  address = address * 2 + EEPROM_START_ADDRESS;
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data);
  return status;
}

HAL_StatusTypeDef flash_write_4byte(uint32_t address, uint32_t data) {
  HAL_StatusTypeDef status;
  address = address * 4 + EEPROM_START_ADDRESS;
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
  return status;
}

uint16_t flash_read_2byte(uint32_t address) {
  uint16_t val = 0;
  address = address * 2 + EEPROM_START_ADDRESS;
  val = *(__IO uint16_t *)address;
  return val;
}

uint32_t flash_read_4byte(uint32_t address) {
  uint32_t val = 0;
  address = address * 4 + EEPROM_START_ADDRESS;
  val = *(__IO uint32_t *)address;
  return val;
}
