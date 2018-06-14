#include <Arduino.h>
#include "AD7714_arduinoUNO.h"

const int RESET = 8;  //pin number Reset
const int CS = 10;  //pin number as Chip select


AD7714_UNO::ResetRoutine(){
    digitalWrite(RESET, LOW);
    delay(100);
    digitalWrite(RESET, HIGH);
    digitalWrite(CS, LOW);
}

AD7714_UNO::writeByteRegister( byte reg, byte value, byte ain){ // only valid for 8 bit registers 0:Communication reg, 1:Mode reg, 2:Filter high, 3:Filter low, 4:Test reg
  byte result = 0;
  if (reg<5) { // byte registers
    byte cmd = 0; // a place to build the correct comm reg value.
    cmd = (reg<< 4); // set the correct RS2..RS0 bits of COMM register. Shift 4-7bit to 0-3bit.
    if(reg==0) { // update global ain Value
      ain = value &7;  //mask for the channel select(CH0-CH8..refer to top of this code)
    }
    cmd = cmd | ain; // keep the analog mux what it was previously configured as.
    byte stat = SPI.transfer(cmd); // actually send the cmd and read the current status
    if(reg!=0){ // actually send the value to reg
      byte unk = SPI.transfer(value);
    }
    result = stat; // return value received, drdy is bit 7.
  }
  return result;
}
//test 2 3 4 5
