#include "stdint.h"
#include "CONFIG.hpp"
#include "Motor.hpp"
#include "Utils.hpp"
#include "I2C.hpp"


void I2C_onReceive(uint8_t data);


// Position and angle is in um and urad (micro-meters and micro-radian)
int32_t positionX, positionY, angle;
// Velocity and angular velocity in um/s and urad/s
int32_t velocity, rotation_velocity;

const int16_t distancePerTick = 1000L * ODO_WHEEL_D_MM * PI / ODO_TICK_PER_TURN;

int8_t roueCodeuseDTick, roueCodeuseGTick;
uint32_t odometryTick;
int16_t delta_distance, delta_angle, delta_X, delta_Y;

static const int32_t delta_t_us = (1000000/ODO_TICK_SPEED);


#ifdef DEBUG
uint16_t odometryMeanTime;
#endif



bool power_enable, aru_state;





void setup() {

  Serial.begin(SERIAL_SPEED);
  I2C_Init(I2C_onReceive);
  
  initGPIO();




  attachInterrupt(digitalPinToInterrupt(PIN_CODEUR_D_PULSE), ISR_codeurD, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_CODEUR_G_PULSE), ISR_codeurG, FALLING);


#ifdef FORCE_POWER_ENABLE
  digitalWrite(PIN_POWER_ENABLE, true);
  power_enable = true;
#endif
  
  positionX = positionY = angle = velocity = rotation_velocity = 0;
  roueCodeuseDTick = roueCodeuseGTick = odometryTick = 0;

  aru_state = false;
  
  configureTimer();
}


void loop() {
#ifdef DEBUG
  Serial.print("\n\n  --== Loop ==--\n\n");
#endif

  // Check battery level
  updateBatteryLevel();
  aru_state = getARU();



  

  if(aru_state){
    power_enable = false;
  }

  // Check if need to turn ON power
  bool state = false;
#ifdef FORCE_POWER_ENABLE
  #ifdef IGNORE_BATTERY_LEVEL
    state = true;
  #else
    state = !low_battery;
  #endif
#else
  #ifdef IGNORE_BATTERY_LEVEL
    state = power_enable;
  #else
    if(!low_battery){
      state = power_enable;
    }
  #endif
#endif

  digitalWrite(PIN_POWER_ENABLE, state);





#ifdef DEBUG
  uint32_t tps = 1000 * odometryTick / millis();
  Serial.print(F("Position : \n  X : "));
  Serial.print(positionX/1000L);
  Serial.print(F(" mm\n  Y : "));
  Serial.print(positionY/1000L);
  Serial.print(F(" mm\n  Angle : "));
  Serial.print(angle/1000L*180L/PI);
  Serial.println(F(" mdeg\n"));
  
  Serial.print(F("Linear velocity : "));
  Serial.print(velocity/1000L);
  Serial.print(F(" mm/s\nAngular velocity : "));
  Serial.print(rotation_velocity/1000L);
  Serial.println(F(" mrad/s\n"));
  
  Serial.print(F("Odometry tick count : "));
  Serial.print(odometryTick);
  Serial.print(F(" (Tick/s : "));
  Serial.print(tps);
  Serial.print(F(")\nOdometry mean time : "));
  Serial.print(odometryMeanTime);
  Serial.print(F(" µs ("));
  Serial.print(odometryMeanTime / (10000.0/ODO_TICK_SPEED) );
  Serial.println(F("%)\n"));

  Serial.print(F("ARU state : "));
  Serial.println(aru_state?F("STOP !"):F("Power enable"));
  Serial.print(F("Power state : "));
  Serial.println(power_enable?F("On"):F("Off"));
  
  Serial.print(F("\nBattery : "));
  Serial.print(battery_level);
  Serial.print(F("mV"));
  if(low_battery)
    Serial.print(F(" !!! LOW BATTERY !!!"));
  Serial.println();




#endif


  delay(100);
  updateMotors();

  

}




void odometryUpdate(){
#ifdef DEBUG
  odometryMeanTime = micros();
#endif
  ++odometryTick;
  
  /*
  int32_t delta_distance = (roueCodeuseDTick + roueCodeuseGTick) * distancePerTick / 2L;
  int32_t delta_angle = 1000L * (roueCodeuseDTick - roueCodeuseGTick) * distancePerTick / (ODO_ENTRAXE_MM);
  
*/

  delta_distance = ((roueCodeuseDTick + roueCodeuseGTick) * distancePerTick) / 2L;
  /*
  delta_angle = 1000L * (roueCodeuseDTick - roueCodeuseGTick) * distancePerTick / (ODO_ENTRAXE_MM);
  delta_angle /= 2L;*/
  delta_angle = 500L * (roueCodeuseDTick - roueCodeuseGTick) * distancePerTick / (ODO_ENTRAXE_MM);
  
  angle += delta_angle;
  delta_X = delta_distance * cos(angle/1000000.0);
  delta_Y = delta_distance * sin(angle/1000000.0);
  angle += delta_angle;

  
  positionX += delta_X;
  positionY += delta_Y;

  
  
  velocity = (velocity * 30L + delta_distance * delta_t_us) / 31L;
  rotation_velocity = (rotation_velocity * 30L + delta_angle * 2L * delta_t_us) / 31L;


  
  roueCodeuseDTick = roueCodeuseGTick = 0;
  
#ifdef DEBUG
  odometryMeanTime = micros() - odometryMeanTime;
#endif
}






void I2C_onReceive(uint8_t data){

#ifdef I2C_DEBUG
    Serial.print(F("Data received from I2C : "));
    Serial.println(data);
#endif


  switch(data){
  case I2C_COMMAND__STOP_ALL:
    reset();
    power_enable = false;
    digitalWrite(PIN_POWER_ENABLE, false);
    setMotorSpeed(0, 0);
    break;
  case I2C_COMMAND__ENABLE_POWER:
    power_enable = true;
    digitalWrite(PIN_POWER_ENABLE, true);
    break;
  case I2C_COMMAND__DISABLE_POWER:
    power_enable = false;
    digitalWrite(PIN_POWER_ENABLE, false);
    break;
  case I2C_COMMAND__GET_BATTERY_LVL:
    I2C_Write(battery_level);
    I2C_Write(battery_level >> 8);
    break;
  case I2C_COMMAND__GET_POSITION:
    I2C_Write(positionX/1000L);
    I2C_Write((positionX/1000L) >> 8);
    I2C_Write(positionY/1000L);
    I2C_Write((positionY/1000L) >> 8);
    I2C_Write(angle/1000L);
    I2C_Write((angle/1000L) >> 8);
    break;
  case I2C_COMMAND__SET_MOTOR_SPEED:
    uint8_t dir = I2C_Read();
    int16_t motorL = (int16_t)I2C_Read();
    int16_t motorR = (int16_t)I2C_Read();
    if(!(dir &0x01)){
      motorL = -motorL;
    }
    if(!(dir &0x02)){
      motorR = -motorR;
    }
    
    break;
  default:

    break;
  }
}




void ISR_codeurD(){
  bool dir = digitalRead(PIN_CODEUR_D_DIR);
  roueCodeuseDTick += dir?-1:1;
#ifdef CODEUR_DEBUG
  Serial.print("Codeur D : ");
  Serial.print(dir?"UP":"DOWN");
  Serial.print(" = ");
  Serial.println(roueCodeuseDTick);
#endif

}

void ISR_codeurG(){
  bool dir = digitalRead(PIN_CODEUR_G_DIR);
  roueCodeuseGTick += dir?1:-1;
#ifdef CODEUR_DEBUG
  Serial.print("Codeur G : ");
  Serial.print(dir?"UP":"DOWN");
  Serial.print(" = ");
  Serial.println(roueCodeuseGTick);
#endif
}
