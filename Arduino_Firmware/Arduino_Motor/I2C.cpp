#include "I2C.hpp"

#define I2C_QUEUE_LENGTH 64

void (*receiveHandler)(uint8_t) = NULL;

uint8_t I2C_queueToSend[I2C_QUEUE_LENGTH];
uint8_t I2C_queueRIndex = 0, I2C_queueWIndex = 0;

void I2C_onReceive(int argc);
void I2C_onRequest();

void I2C_Init(void (*onReceiveHandler)(uint8_t)){
  Wire.begin(I2C_ADDRESS);
  Wire.setClock(100000);
  Wire.onReceive(I2C_onReceive);
  Wire.onRequest(I2C_onRequest);
  receiveHandler = onReceiveHandler;
}


bool I2C_queueHasNewElt(){
  return I2C_queueRIndex != I2C_queueWIndex;
}

void I2C_writeDataToQueue(uint8_t data){
  I2C_queueToSend[I2C_queueWIndex++] = data;
  if(I2C_queueWIndex >= I2C_QUEUE_LENGTH)
    I2C_queueWIndex = 0;
}




uint8_t I2C_readDataFromQueue(){
  if(!I2C_queueHasNewElt())
    return 0;
  
  uint8_t data = I2C_queueToSend[I2C_queueRIndex++];
  if(I2C_queueRIndex >= I2C_QUEUE_LENGTH)
    I2C_queueRIndex = 0;
    
  return data;
}


void I2C_onReceive(int argc){
  uint8_t data;
  while(Wire.available()){
    data = Wire.read();
    if(receiveHandler != NULL)
      receiveHandler(data);
      
  }
}


void I2C_onRequest(){
  uint8_t data;
  if(I2C_queueHasNewElt())
    data = I2C_readDataFromQueue();
  else
    data = 0xA5;
  
  Wire.write(data);
  
#ifdef I2C_DEBUG
  Serial.print(F("Data sent throught I2C : "));
  Serial.println(data);
#endif
}


void I2C_Write(uint8_t data){
  I2C_writeDataToQueue(data);
}

uint8_t I2C_Read(){
  if(Wire.available() > 0)
    return Wire.read();

  return 0;
}
