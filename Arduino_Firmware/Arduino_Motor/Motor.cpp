#include "Motor.hpp"


int16_t motorGTarget, motorDTarget;
int16_t motorGAct, motorDAct;


void setMotorSpeed(int16_t speedG, int16_t speedD){
  if(speedG > MOTOR_MAX_SPEED)
    speedG = MOTOR_MAX_SPEED;
  if(speedG < -MOTOR_MAX_SPEED)
    speedG = -MOTOR_MAX_SPEED;
    
  if(speedD > MOTOR_MAX_SPEED)
    speedD = MOTOR_MAX_SPEED;
  if(speedD < -MOTOR_MAX_SPEED)
    speedD = -MOTOR_MAX_SPEED;

  motorGTarget = speedG;
  motorDTarget = speedD;
  
}



void setDMotor(uint8_t dir, uint8_t speed){
  if(speed > MOTOR_MAX_SPEED)
    speed = MOTOR_MAX_SPEED;
  
  digitalWrite(PIN_MOTOR_D_DIR, dir);
  analogWrite(PIN_MOTOR_D_PWM, speed);
}

void setGMotor(uint8_t dir, uint8_t speed){
  if(speed > MOTOR_MAX_SPEED)
    speed = MOTOR_MAX_SPEED;
    
  digitalWrite(PIN_MOTOR_G_DIR, dir);
  analogWrite(PIN_MOTOR_G_PWM, speed);
}


uint32_t nextMotorUpdate = 0;

void updateMotors(){

  if(nextMotorUpdate > millis())
    return;

  nextMotorUpdate = millis()+10;

  if(motorGAct < motorGTarget){
    motorGAct += MOTOR_ACC_INC;
    if(motorGAct > motorGTarget)
      motorGAct = motorGTarget;
  }
  if(motorGAct > motorGTarget){
    motorGAct -= MOTOR_ACC_INC;
    if(motorGAct < motorGTarget)
      motorGAct = motorGTarget;
  }

  if(motorDAct < motorDTarget){
    motorDAct += MOTOR_ACC_INC;
    if(motorDAct > motorDTarget)
      motorDAct = motorDTarget;
  }
  if(motorDAct > motorDTarget){
    motorDAct -= MOTOR_ACC_INC;
    if(motorDAct < motorDTarget)
      motorDAct = motorDTarget;
  }
  

#ifdef MOTOR_DEBUG

  Serial.print(F("Motor D : Target( "));
  Serial.print(motorDTarget);
  Serial.print(F(")  Act( "));
  Serial.print(motorDAct);
  Serial.print(F(")\nMotor G : Target( "));
  Serial.print(motorGTarget);
  Serial.print(F(")  Act( "));
  Serial.print(motorGAct);
  Serial.print(F(")\n\n"));
  
#endif
  

  

  if(motorGAct < 0){
    setGMotor(1, -motorGAct);
  }else{
    setGMotor(0, motorGAct);
  }

  if(motorDAct < 0){
    setDMotor(1, -motorDAct);
  }else{
    setDMotor(0, motorDAct);
  }
  
  
}
