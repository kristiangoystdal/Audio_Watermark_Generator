// flash_flag.h
#ifndef FLASH_FLAG_H
#define FLASH_FLAG_H

#include <stdbool.h>
#include <stdint.h>

void FlashFlag_SetTimeWasSet(void);
bool FlashFlag_TimeWasSet(void);

#endif