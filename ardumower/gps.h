#include "arduino.h"
#ifndef gps_h
#define gps_h
class GPS
{
  public:
    void init();
    void run();
    void writePi(String stringLine);
    
  private:
    char buf[120];
    uint8_t pos;
};

#endif

