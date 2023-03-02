#ifndef __CONFIG_H__
#define __CONFIG_H__





// Debug configuration

// #define DEBUG
// #define ODOMETRY_DEBUG
// #define CODEUR_DEBUG
// #define FORCE_POWER_ENABLE
// #define IGNORE_BATTERY_LEVEL
// #define I2C_DEBUG
// #define MOTOR_DEBUG



// Serial speed 

#define SERIAL_SPEED  1000000




// Pinout 

#define PIN_CODEUR_D_PULSE  2
#define PIN_CODEUR_G_PULSE  3
#define PIN_CODEUR_D_DIR    4
#define PIN_CODEUR_G_DIR    5
#define PIN_MOTOR_G_PWM     6
#define PIN_MOTOR_G_DIR     7
#define PIN_MOTOR_D_DIR     8
#define PIN_MOTOR_D_PWM     9
#define PIN_POWER_ENABLE    11
#define PIN_ARU_SIGNAL      10
#define PIN_LED             12
#define PIN_BATTERY_LEVEL   A0
#define PIN_I2C_SDA         A4
#define PIN_I2C_SCL         A5




// Battery low level in mV

#define BATTERY_LOW_THRESHOLD 13000 




// I2C slave address of this Arduino

#define I2C_ADDRESS 12




// Motor acceleration ramp and max speed

#define MOTOR_ACC_INC 2000
#define MOTOR_MAX_SPEED 100




// List of all I2C commands for this Arduino

#define I2C_COMMAND__STOP_ALL         0
#define I2C_COMMAND__ENABLE_POWER     1
#define I2C_COMMAND__DISABLE_POWER    2
#define I2C_COMMAND__GET_BATTERY_LVL  3
#define I2C_COMMAND__GET_POSITION     4
#define I2C_COMMAND__SET_MOTOR_SPEED  5




// Odometry config

#define ODO_ENTRAXE_MM     235
#define ODO_WHEEL_D_MM     35
#define ODO_TICK_PER_TURN  512

// Values : 500, 1000, 2000, 4000, 8000
#define ODO_TICK_SPEED     1000




// Other config

#define BATTERY_SMOOTH  0.8




#endif // __CONFIG_H__
