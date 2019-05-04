#ifndef __KEY_H
#define __KEY_H


#include "main.h"
#include "stm32f1xx_hal.h"


uint8_t ScanKey_Begin(void);
uint8_t ScanKey_Title(void);
uint8_t ScanKey_Bat(void);
uint8_t ScanKey_Set(void);

uint8_t ScanKey_LightAdd(void);
uint8_t ScanKey_LightSub(void);
uint8_t ScanKey_ScreenCloseAdd(void);
uint8_t ScanKey_ScreenCloseSub(void);
uint8_t ScanKey_TurnOffAdd(void);
uint8_t ScanKey_TurnOffSub(void);

#endif
















