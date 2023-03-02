#ifndef __I2C_LIB_H__
#define __I2C_LIB_H__


#include "CONFIG.hpp"
#include <Wire.h>
#include "Arduino.h"


#define I2C_QUEUE_LENGTH 64

void I2C_Init(void (*onReceiveHandler)(uint8_t));
void I2C_Write(uint8_t data);
uint8_t I2C_Read();







#endif // __I2C_LIB_H__
