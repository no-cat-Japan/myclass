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



#include <SPI.h>
#include "AD7714_arduinoUNO.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// define channels AD7714
#define CH0 0x00 // Pseudo Differential AIN1 - AIN6
#define CH1 0x01 // Pseudo Differential AIN2 - AIN6
#define CH2 0x02 // Pseudo Differential AIN3 - AIN6
#define CH3 0x03 // Pseudo Differential AIN4 - AIN6
#define CH4 0x04 // Fully Differential AIN1 - AIN2
#define CH5 0x05 // Fully Differential AIN3 - AIN4
#define CH6 0x06 // Fully Differential AIN5 - AIN6
#define CH7 0x07 // Test Mode AIN6 - AIN6

#define CRr 0x08 //Controll Register read mode mask

// Gain
byte Gain1=0; // Data Register
/* 0 -> x1
   1 -> x2
   2 -> x4
   3 -> x8
   4 -> x16
   5 -> x32
   6 -> x64
   7 -> x128
 */

//bipolar or unipolar  FHreg: B/U W/L BST ZERO    FS11 FS10 FS9 FS8
byte Bip=0x20;
byte Unip=0xa0;
byte ADpolar=0;

// constants and variables:
const int dataReady = 9; //pin number as DRDY
//const int DataRead = 7; //pin number as a hard switch for the data read
const int CS = 10;  //pin number as Chip select
const int RESET = 8;  //pin number Reset
const int But1 = 7;  //pin number for the button1

byte ain;  //input pin
/* CH0-CH7 as follows
 CH0 Pseudo Differential AIN1 - AIN6
 CH1 Pseudo Differential AIN2 - AIN6
 CH2 Pseudo Differential AIN3 - AIN6
 CH3 Pseudo Differential AIN4 - AIN6
 CH4 Fully Differential AIN1 - AIN2
 CH5 Fully Differential AIN3 - AIN4
 CH6 Fully Differential AIN5 - AIN6
 CH7 Test Mode AIN6 - AIN6
*/

bool bits24 = false; // is dataregister configured for 24bits?

byte FHhi;  // high bits of the filter High register

//12bit filter value
int filt;
byte filtFH=0;
byte filtFL=0;

// voltage value
float BaseV=0.0;
float RefV=0.0;  //Reference Voltage
unsigned long DatV=0;
float ResultV=0.0;

AD7714_UNO AD7714;

void setup() {
//program parameter
  ADpolar=Bip;  //bipolar mode(Bip) or unipolar mode(Unip)
  RefV=2.489;  //Reference Voltage 2.489
//  RefV=1.2;  //Reference Voltage
  bits24 = 1; //1(24bit) or 0(16bit) (AD rate)
  filt=1953;
//  filt=781;
//  filt=312;
  /* 19~4000 value of the filter (input to FS11~FS0 bit of the filter high/low registar)
  Filter First Notch =(Fclkin/128)/code   :Fclkin=1MHz
  filt(code)=1953 -> FFN=4Hz
  filt(code)=781 -> FFN=10Hz
  filt(code)=312 -> FFN=25Hz
  */

  if(bits24){
    BaseV=RefV*1000000/16777215; // to makes the calculation simple
    FHhi=ADpolar|0x40;
  } else {
    BaseV=RefV*1000000/65535; // to makes the calculation simple
    FHhi=ADpolar;
  }

  // -------- LCD initialization
  lcd.init(); // initialize the lcd
  lcd.init();
  lcd.backlight();
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 1);

  // -------- SPI initialization
  pinMode(CS, OUTPUT);  // Set the SS pin as an output
  digitalWrite(CS, HIGH);  // Set the SS pin HIGH
  SPI.begin();  // Begin SPI hardware
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(210);  // Slow down SPI clock
  SPI.setDataMode(SPI_MODE3);

  // initalize the  data ready and chip select pins:
  pinMode(dataReady, INPUT);
  pinMode(RESET, OUTPUT);
  pinMode(But1, INPUT_PULLUP);

  // calculate the filter value
  for(byte i=0;i<4;i++){
    if((filt>>(11-i)&1)==1){
      filtFH=filtFH|(B00000001<<(3-i));
    }
  }
  for(byte i=0;i<8;i++){
    if((filt>>(7-i)&1)==1){
      filtFL=filtFL|(B00000001<<(7-i));
    }
  }
  AD7714.ResetRoutine();
  lcd.setCursor(0,0);
  lcd.print("push button");
}


float Avarage(){
  byte CollectNo=10;  //sample number
  byte TryNo=CollectNo*3;  //try number is sample number * 3
  unsigned long PrimNo[TryNo+1];
  PrimNo[0]=0;
  unsigned long tempPN; //temporary place to sort
  float result=0.0;
// get AD results and sort
  for(byte i=0;i<TryNo;i++){
    delay(200);
    //sort
    PrimNo[i]=AD7714.readADresult(ain, bits24);
    for(byte j=0;j<i;j++){
      if(PrimNo[i-j]>PrimNo[i-j-1]){
        tempPN=PrimNo[i-j-1];
        PrimNo[i-j-1]=PrimNo[i-j];
        PrimNo[i-j]=tempPN;
      }
    }
  }
  for(byte k=CollectNo;k<(TryNo-CollectNo);k++){
    result=result+PrimNo[k+1];
  }
  result=result/(TryNo-CollectNo*2);
    
  if(ADpolar==Bip){
    result=BaseV*2*result;
    result=result-RefV*1000000;
  } else {
    result=BaseV*result;
  }
  return result;
}

void loop() {
  if(!digitalRead(But1)){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("CH4 wait");
    lcd.setCursor(0,0);
    ain = CH4;
    AD7714.InitRoutine(FHhi, filtFH, filtFL, ain);
    lcd.print(Avarage());
    lcd.print("uV");
    lcd.setCursor(0,1);
    lcd.print("CH6 wait");
    lcd.setCursor(0,1);
    ain = CH6;
    AD7714.InitRoutine(FHhi, filtFH, filtFL, ain);
    lcd.print(Avarage());
    lcd.print("uV");
    lcd.setCursor(10,0);
    lcd.print("push");
    lcd.setCursor(10,1);
    lcd.print("button");
    lcd.setCursor(0,0);
  }
}
//commit test
