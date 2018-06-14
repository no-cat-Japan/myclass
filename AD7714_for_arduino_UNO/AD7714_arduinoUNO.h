#ifndef _AD7714_arduinoUNO_h
#define _AD7714_arduinoUNO_h
#include <SPI.h>


class AD7714_UNO{
  public:
    ResetRoutine();
    writeByteRegister(byte reg, byte value, byte ain);
};

#endif
