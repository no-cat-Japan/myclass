#ifndef _AD7714_arduinoUNO_h
#define _AD7714_arduinoUNO_h
#include <SPI.h>


class AD7714_UNO{
  public:
    ResetRoutine();
    byte writeByteRegister(byte reg, byte value, byte ain);
    unsigned long readbigRegister( byte reg, byte ain, bool bits24);
    void InitRoutine(byte FHhi, byte filtFH, byte filtFL, byte ain);
    unsigned long readADresult(byte ain, bool bits24);
};

#endif
