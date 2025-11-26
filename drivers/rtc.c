#include "rtc.h"

static const uint16_t kMAGIC = 0x32F2;
static const uint32_t kMAGIC_ADDR = RTC_BKP_DR19;

// Configures the RTC for use with an external 32.768KHz crystal
// Returns whether this was the first run or not
bool RTC_Config(void) {
  // Configuration data placeholders
  RTC_DateTypeDef RTC_DateStructure;
  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_InitTypeDef RTC_InitStructure;

  // Turn on backup power domain clock and enable register access
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  // Enable backup register access
  PWR_BackupAccessCmd(ENABLE);

  // Read the last backup register and check for a magic number
  // Configure the RTC from scratch if necessary
  if (RTC_ReadBackupRegister(kMAGIC_ADDR) != kMAGIC) {
    // Turn on the low-speed external oscillator and wait for it to settle
    RCC_LSEConfig(RCC_LSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
    // Select the LSE and enable the real time clock
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();
    // Configure division of 32.768 KHz down to 1 Hz
    RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
    RTC_InitStructure.RTC_SynchPrediv = 0xFF;
    RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
    // Apply changes to the RTC peripheral
    RTC_Init(&RTC_InitStructure);
    // Set the date: Tuesday, 2013-01-01
    RTC_DateStructure.RTC_Year = 13;
    RTC_DateStructure.RTC_Month = RTC_Month_January;
    RTC_DateStructure.RTC_Date = 1;
    RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Tuesday;
    RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
    // Set the time to midnight
    RTC_TimeStructure.RTC_H12 = RTC_H12_AM;
    RTC_TimeStructure.RTC_Hours = 0x00;
    RTC_TimeStructure.RTC_Minutes = 0x00;
    RTC_TimeStructure.RTC_Seconds = 0x00;
    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
    // Save a magic short to the backup registers
    RTC_WriteBackupRegister(kMAGIC_ADDR, kMAGIC);
    return true;
  } else {
    RTC_WaitForSynchro();
    return false;
  }
}

// Sets the RTC time from a tm struct
void RTC_SetTimeFromStruct(struct tm* t) {
  RTC_DateTypeDef date;
  RTC_TimeTypeDef time;
  // Rehydrate the date
  date.RTC_Year = t->tm_year - 100;
  date.RTC_Month = t->tm_mon + 1;
  date.RTC_Date = t->tm_mday;
  date.RTC_WeekDay = (t->tm_wday == 0) ? 7 : t->tm_wday;
  RTC_SetDate(RTC_Format_BIN, &date);
  // Rehydrate the time
  time.RTC_Hours = t->tm_hour;
  time.RTC_Minutes = t->tm_min;
  time.RTC_Seconds = t->tm_sec;
  RTC_SetTime(RTC_Format_BIN, &time);
}

// Gets RTC time and populates a tm struct
void RTC_GetTimeToStruct(struct tm* t) {
  RTC_DateTypeDef date;
  RTC_TimeTypeDef time;
  // Pull the date and time from the RTC
  RTC_GetTime(RTC_Format_BIN, &time);
  RTC_GetDate(RTC_Format_BIN, &date);
  // Turn them in to a tm struct
  t->tm_year = date.RTC_Year + 100;
  t->tm_mon = date.RTC_Month - 1;
  t->tm_mday = date.RTC_Date;
  t->tm_wday = date.RTC_WeekDay % 7;
  t->tm_hour = time.RTC_Hours;
  t->tm_min = time.RTC_Minutes;
  t->tm_sec = time.RTC_Seconds;
}
