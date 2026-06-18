#include "flash_flag.h"
#include "stm32g4xx_hal.h"

// Last page of 128KB flash on STM32G431
#define FLASH_FLAG_ADDR 0x0801F800UL

#define FLAG_TIME_SET 0x00000001UL
#define FLAG_EMPTY 0xFFFFFFFFUL

bool FlashFlag_TimeWasSet(void) {
  uint32_t val = *(__IO uint32_t *)FLASH_FLAG_ADDR;
  return (val == FLAG_TIME_SET);
}

void FlashFlag_SetTimeWasSet(void) {
  if (FlashFlag_TimeWasSet())
    return; // already set, no need to erase

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = 63; // last page of 128KB flash (64 pages x 2KB)
  erase.NbPages = 1;
  HAL_FLASHEx_Erase(&erase, &page_error);

  // G4 programs in double-words (64-bit)
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_FLAG_ADDR,
                    (uint64_t)FLAG_TIME_SET);

  HAL_FLASH_Lock();
}