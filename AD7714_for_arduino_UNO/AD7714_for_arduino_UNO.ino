/* Hardware Connections AD7714 <--> Arduino
PIN 1 -> SCLK : external clock SPI Arduino <-----> Arduino Digital 13
PIN 2 -> MCLK IN : 1 MHz by crystal (input)
PIN 3 -> MCLK OUT : 1 MHz (output)
PIN 4 -> POL = 0: first transition of the serial clock in a data transfer operation is from a low to a high
PIN 5 -> (SYNC) = 1 : synchronization of the digital filters and analog modulators
PIN 6 -> (RESET) : Reset (PIN 30) Arduino <-----> Arduino Digital 8
PIN 7/8/9/10 -> AIN1/AIN2/AIN3/AIN4 : Analog Input Channel
PIN 11 -> (STANDBY) = 1 : disable Standby
PIN 12 -> AVdd = 3.3 V
PIN 13 -> BUFFER = 0 : analog input is shorted out
PIN 14 -> REF IN(-) = AGND : negative input of the differential reference input
PIN 15 -> REF IN(+) = AVdd/2 : positive input of the differential reference input
PIN 16/17 -> AIN5/AIN6 : Analog Input Channel
PIN 18 -> AGND = GND
PIN 19 -> (CS) : chip select SPI (PIN 52) Arduino <-----> Arduino Digital 10
PIN 20 -> (DRDY) : logic input of the AD7714 <-----> Arduino Digital 9
PIN 21 -> DOUT : serial data output, MISO SPI Arduino <-----> Arduino Digital 12
PIN 22 -> DIN : serial data input, MOSI SPI Arduino <-----> Arduino Digital 11
PIN 23 -> DVdd = 3.3 V
PIN 24 -> DGND = GND
*/
/* Hardware Connections Arduino <--> AD7714,74HC14,ADM3202,1602 I2C
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

// SPI library
#include <SPI.h>
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

#define CRr 0x08 //Controll Register read mode mask

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

bool bits24 = false; // is dataregister configured for 24bits?

byte FHhi;  // high bits of the filter High register

//12bit filter value
int filt=0;
byte filtFH=0;
byte filtFL=0;

// voltage value
float BaseV=0.0;
float RefV=0.0;  //Reference Voltage
unsigned long DatV=0;
float ResultV=0.0;

void setup() {
//program parameter
  ADpolar=Bip;  //bipolar mode(Bip) or unipolar mode(Unip)
  RefV=2.489;  //Reference Voltage 2.492
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
  ain = CH4; 
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
//  pinMode(DataRead, INPUT_PULLUP);
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
                            ResetRoutine();
                            InitRoutine();
  lcd.setCursor(0,0);
  lcd.print("push button");
}

void ResetRoutine(){
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  digitalWrite(CS, LOW);
}

void InitRoutine(){
  writeByteRegister(FHRg, FHhi|filtFH);  //set the filterHigh registar
  writeByteRegister(FLRg, filtFL);  //set the filterLow registar

  writeByteRegister(MoRg, (SlfCali<<5)|(Gain1<<2));  //Self Calibration
  while(digitalRead(dataReady)); //
}

byte writeByteRegister( byte reg, byte value){ // only valid for 8 bit registers 0:Communication reg, 1:Mode reg, 2:Filter high, 3:Filter low, 4:Test reg
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
//      if(reg==2) bits24=(value &0x40)==0x40; // if configured for 24bit data register
    }
    result = stat; // return value received, drdy is bit 7.
  }
  return result;
}

unsigned long readbigRegister( byte reg){ // only valid for 16,24 bit registers 5..7
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

float Avarage(){
  byte GetCount=0;
  byte CollectNo=10;  //change the sample number
  byte TryNo=CollectNo*3;
  unsigned long PrimNo[TryNo+1];
  PrimNo[0]=0;
  unsigned long tempPN; //temporary place to sort
  unsigned long GetResult=0;
  float result=0.0;
  unsigned long CompL=0;
  unsigned long CompH=0;
  if(bits24){
//    CompL=4300;
//    CompH=5500;
//    CompL=43000;
//    CompH=55000;
//    CompL=30;
    CompL=8300000;
//    CompH=16777300;
    CompH=8500000;
 } else {
    CompL=17;
    CompH=22;
  }
  GetCount=0;
  lcd.setCursor(0,1);
  lcd.print("---------");
//  for(byte i=0;i<10;i++){
// get AD results for TryNo
  for(byte i=0;i<TryNo;i++){
    delay(200);
    GetResult=readbigRegister(DtRg);
    //sort
    PrimNo[i]=GetResult;
    for(byte j=0;j<i;j++){
      if(PrimNo[i-j]>PrimNo[i-j-1]){
        tempPN=PrimNo[i-j-1];
        PrimNo[i-j-1]=PrimNo[i-j];
        PrimNo[i-j]=tempPN;
      }
    }
    lcd.setCursor(5,0);
    lcd.print("       ");
    lcd.setCursor(5,0);
    lcd.print(GetResult);
//      if((GetResult<4300)|(5500<GetResult)){  //around 35degrees Celsius, 24bit, Unipolar
    if((GetResult<CompL)|(CompH<GetResult)){  //around 35degrees Celsius, 16bit, Unipolar
      lcd.setCursor(i,1);
      lcd.print("x");
      GetResult=0;
    } else {
//      result=result+GetResult;
      GetCount++;
//      lcd.setCursor(i,1);
      lcd.setCursor(0,1);
      lcd.print("o");
    }
//      result=result+readbigRegister(DtRg);  //get the AD value as 16 or 24bit
//      lcd.print(".");
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
  lcd.clear();
  lcd.setCursor(0,1);
  if(GetCount<5){
    lcd.setCursor(0,0);
    lcd.print("wait ");
    lcd.print("error!");
    lcd.setCursor(0,1);
  }
  return result;
}

void loop() {
  if(!digitalRead(But1)){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("wait");
    lcd.setCursor(0,1);
    lcd.print(Avarage());
    lcd.print("uV");
    lcd.setCursor(0,0);
    lcd.print("push button");
  }
}

