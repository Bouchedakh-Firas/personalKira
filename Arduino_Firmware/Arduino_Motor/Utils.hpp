#ifndef __UTILS_H__
#define __UTILS_H__


#include "stdint.h"
#include "CONFIG.hpp"
#include "Arduino.h"


#define getARU() (!digitalRead(PIN_ARU_SIGNAL))


extern uint16_t battery_level;
extern bool low_battery;

void configureTimer();
void initGPIO();
void updateBatteryLevel();
void reset();


extern void odometryUpdate();


#endif // __UTILS_H__
