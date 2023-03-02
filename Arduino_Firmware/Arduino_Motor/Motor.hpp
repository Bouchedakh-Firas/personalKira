#ifndef __MOTOR_H__
#define __MOTOR_H__


#include "stdint.h"
#include "CONFIG.hpp"
#include "Arduino.h"



void setMotorSpeed(int16_t speedG, int16_t speedD);
void updateMotors();






#endif // __MOTOR_H__
