/* Hardware Connections AD7714<--> Arduino UNO , AD780
AD7714 PIN1 (SCLK) <----> Arduino UNO 13(D13)
AD7714 PIN2 (MCLK_IN) <---
                         |
                        1MHz
                         |
AD7714 PIN3 (MCLK_OUT) <--
AD7714 PIN4 (POL) <----> 3.3V
AD7714 PIN5 (SYNC) <----> 3.3V
AD7714 PIN6 (RESET) <----> Arduino UNO Digital 8(D8)
AD7714 PIN7 (AIN1) <----> (Thermo couple)
AD7714 PIN8 (AIN2) <----> (Thermo couple)
AD7714 PIN9 (AIN3) <----> (Thermo couple)
AD7714 PIN10 (AIN4) <----> (Thermo couple)
AD7714 PIN11 (STANDBY) <-----> 3.3V
                          |
AD7714 PIN12 (AVDD) <-----|
                          |
                         0.1uF
                          |
                         GND
AD7714 PIN13 (BUFFER) <----> AD7714 PIN14 (REFIN(-)) <----> GND
                        |        |
                      0.1uF    10uF
                        |        |
AD7714 PIN15 (REFIN(+)) <-----------------> AD780 PIN6 (VOUT)
AD7714 PIN16 (AIN5) <----> (Thermo couple)
AD7714 PIN17 (AIN6) <----> (Thermo couple)
AD7714 PIN18 (AGND) <----> GND
AD7714 PIN19 (CS) <----> Arduino UNO Digital 10(D10)
AD7714 PIN20 (DRDY) <----> Arduino UNO Digital 9(D9)
AD7714 PIN21 (DOUT) <----> Arduino UNO Digital 12(D12)
AD7714 PIN22 (DIN) <----> Arduino UNO Digital 11(D11)
AD7714 PIN23 (DVDD) <----> 3.3V
                      |
                     0.1uF
                      |
AD7714 PIN24 (DGND) <----> GND

Arduino Digital 7 <-----> SW
Arduino Digital 8 <-----> AD7714 PIN 6 (RESET)
Arduino Digital 9 <-----> AD7714 PIN 20 (DRDY) : logic input of the AD7714
Arduino Digital 10 <-----> AD7714 PIN 19 (CS) : chip select SPI
Arduino Digital 11 <-----> AD7714 PIN 22 (DIN) : serial data input, MOSI SPI Arduino
Arduino Digital 12 <-----> AD7714 PIN 21 (DOUT) : serial data output, MISO SPI Arduino
Arduino Digital 13 <-----> AD7714 PIN 1 (SCLK) : external clock SPI Arduino
Arduino Digital SDA <-----> 1602 I2C PIN 3 (SDA)
Arduino Digital SCL <-----> 1602 I2C PIN 4 (SCL)
3.3V <-----> AD7714,74HC14,ADM3202
5V <-----> AD780, 1602 I2C
*/

#include <Arduino.h>
#include "AD7714_arduinoUNO.h"

// define Registers
#define CoRg 0 // Communication Register
#define MoRg 1 // Mode Register
#define FHRg 2 // Filter High Register
#define FLRg 3 // Filter Low Register
#define DtRg 5 // Data Register
#define ZCRg 6  // Zero-scale Calibration Register
#define FCRg 7  // Full-scale Calibration Register

// define Calibration
#define SlfCali 1 // Self calibration
#define ZeScCali 2 // Zero scale calibration
#define FlScCali 3 // Full scale calibration
#define SOCali 4 // System ofset calibration

// define Gain
#define Gain1 0 // Data Register
/* 0 -> x1
   1 -> x2
   2 -> x4
   3 -> x8
   4 -> x16
   5 -> x32
   6 -> x64
   7 -> x128
 */
 
const int RESET = 8;  //pin number Reset
const int CS = 10;  //pin number as Chip select
const int dataReady = 9; //pin number as DRDY

AD7714_UNO::ResetRoutine(){
    digitalWrite(RESET, LOW);
    delay(100);
    digitalWrite(RESET, HIGH);
    digitalWrite(CS, LOW);
}

void AD7714_UNO::InitRoutine(byte FHhi,byte filtFH, byte filtFL, byte ain){
  AD7714_UNO::writeByteRegister(FHRg, FHhi|filtFH, ain);  //set the filterHigh registar
  AD7714_UNO::writeByteRegister(FLRg, filtFL, ain);  //set the filterLow registar

  AD7714_UNO::writeByteRegister(MoRg, (SlfCali<<5)|(Gain1<<2), ain);  //Self Calibration
  while(digitalRead(dataReady)); //
}

byte AD7714_UNO::writeByteRegister( byte reg, byte value, byte ain){ // only valid for 8 bit registers 0:Communication reg, 1:Mode reg, 2:Filter high, 3:Filter low, 4:Test reg
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

unsigned long AD7714_UNO::readADresult(byte ain, bool bits24){
  unsigned long result = AD7714_UNO::readbigRegister( DtRg, ain, bits24);
  return result;
}

unsigned long AD7714_UNO::readbigRegister( byte reg, byte ain, bool bits24){ // only valid for 16,24 bit registers 5..7
  unsigned long result = 0;
  if ((reg>4)&&(reg<8)) { // big registers
    byte cmd = 0; // The place to build the correct comm reg value.
    cmd = (reg<< 4); // set the correct RS2..RS0 bits of COMM register
    cmd = cmd | ain; // keep the analog mux what it was previously configured as.
    cmd = cmd | 0x08; // read mode
    byte stat = SPI.transfer(cmd); // actually send the cmd and read the current status
    while(!digitalRead(dataReady));
    byte b = SPI.transfer(0); // send out placeholder get back byte
    result = b;
    b = SPI.transfer(0); // send out placeholder get back bigger byte
    result = (result << 8)+b; // build up 16 bit value from bytes
    if(bits24||(reg>5)){ //do a 24bit transfer
      b = SPI.transfer(0); // send out placeholder get back bigger byte
      result = (result << 8)+ b; // build up 24 bit value from bytes.
    }
  }
  return result;
}

//test 2 3 4 5
