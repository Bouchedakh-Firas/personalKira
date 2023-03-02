#include "Utils.hpp"

uint16_t battery_level;
bool low_battery;



void initGPIO(){
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ARU_SIGNAL, INPUT);
  pinMode(PIN_POWER_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_D_PWM, OUTPUT);
  pinMode(PIN_MOTOR_D_DIR, OUTPUT);
  pinMode(PIN_MOTOR_G_PWM, OUTPUT);
  pinMode(PIN_MOTOR_G_DIR, OUTPUT);
  pinMode(PIN_CODEUR_G_DIR, INPUT);
  pinMode(PIN_CODEUR_D_DIR, INPUT);
  pinMode(PIN_CODEUR_G_PULSE, INPUT);
  pinMode(PIN_CODEUR_D_PULSE, INPUT);
  pinMode(PIN_BATTERY_LEVEL, INPUT);

  digitalWrite(PIN_MOTOR_G_PWM, 0);
  digitalWrite(PIN_MOTOR_D_PWM, 0);
}



void configureTimer(){
  noInterrupts();
  // Initial configuration
  TCCR2A = 0b00000010;
  TCCR2B = 0;
  TIMSK2 = 0b00000010;
  TCNT2 = 0;
  

  // Configure timing
  OCR2A = 249;
#if ODO_TICK_SPEED <= 500
  TCCR2B |= 0b00000101;
#elif ODO_TICK_SPEED <= 1000
  TCCR2B |= 0b00000100;
#elif ODO_TICK_SPEED <= 2000
  TCCR2B |= 0b00000011;
#elif ODO_TICK_SPEED <= 4000
  TCCR2B |= 0b00000011;
  OCR2A = 124;
#else
  TCCR2B |= 0b00000010;
#endif
  interrupts();
}

/** Update the current battery level in milli-volts. */
void updateBatteryLevel(){
  uint32_t value = analogRead(PIN_BATTERY_LEVEL);

  //ADC to Volts
  value =  value * 5000 / 1024;
  
  // Resistor divider
  value = (value * 57) / 10;

  if(battery_level == 0)
    battery_level = value;
  else
    battery_level = (battery_level * BATTERY_SMOOTH) + (value * (1-BATTERY_SMOOTH));


  #ifndef IGNORE_BATTERY_LEVEL
  if(battery_level < BATTERY_LOW_THRESHOLD && !low_battery){
    low_battery = true;
  }else if(battery_level > (BATTERY_LOW_THRESHOLD + 1000) && low_battery){
    low_battery = false;
    digitalWrite(PIN_LED, 0);
  }

  
#ifndef IGNORE_BATTERY_LEVEL
  digitalWrite(PIN_LED, (millis()%500 > 400) && low_battery);
#endif
#endif
  
}


ISR(TIMER2_COMPA_vect){
  TIMSK2 = 0;
  odometryUpdate();
  TIMSK2 = 0b00000010;
}


void reset(){
  extern volatile unsigned long timer0_millis, timer0_overflow_count;
  extern int32_t positionX, positionY, angle, velocity, rotation_velocity;
  extern int8_t roueCodeuseDTick, roueCodeuseGTick;
  extern uint32_t odometryTick;
  extern bool power_enable, aru_state;
  extern int16_t delta_distance, delta_angle;
  extern int16_t motorGTarget, motorDTarget, motorGAct, motorDAct;
  noInterrupts();
  
  timer0_millis = timer0_overflow_count = 0;
  positionX = positionY = angle = velocity = rotation_velocity = 0;
  roueCodeuseDTick = roueCodeuseGTick = 0;
  odometryTick = 0;
  power_enable = aru_state = false;
  delta_distance = delta_angle = 0;

  motorGTarget, motorDTarget, motorGAct, motorDAct = 0;


  initGPIO();
  
  interrupts();
}
