#include "CONFIG.hpp"
#include "I2C.hpp"

void I2C_onReceive(uint8_t data);



void setup() {
  Serial.begin(1000000);
  I2C_Init(I2C_onReceive);
  

}

void loop() {
  delay(1000);
  

}



void I2C_onReceive(uint8_t data){
#ifdef DEBUG
    Serial.print(F("Data received from I2C : "));
    Serial.println(data);
#endif

  static uint8_t counter;
  I2C_Write(counter++);


  
}
