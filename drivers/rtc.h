#pragma once

#include <stm32f4xx.h>
#include <stdbool.h>
#include <time.h>

// Configures the RTC for use with an external 32.768KHz crystal
// Returns whether this was the first run or not
bool RTC_Config(void);

// Sets the RTC time from a tm struct
void RTC_SetTimeFromStruct(struct tm* t);

// Gets RTC time and populates a tm struct
void RTC_GetTimeToStruct(struct tm* t);
