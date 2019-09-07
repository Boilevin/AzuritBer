#include "arduino.h"
#ifndef RpiRemote_h
#define RpiRemote_h
//#define Serial SerialUSB
//class Robot;

class RpiRemote
{
  public:
    
    class Tokeniser
    {
      public:
        Tokeniser(char* str, char token);
        bool next(char* out, int len);

      private:

        char* str;
        char token;
    };



    byte create_checksum(String lineOfString);
    void init();
    void run();
    //void readPi();
    void writePi(String stringLine);
   
  private:
    unsigned long nextTimeRaspberryPISendResetSpeed; // delay between 2 data send to RPI for stat
   
    
    
    bool check_checksum();
    
    byte parse_hex(char h);
    bool encode(char c);
    char buf[120];
    byte pos;
    bool process_buf();
    void read_pfo();
    void readWrite_setting();
    void readWrite_var();
    void receive_command();
    void receive_request();
    void RaspberryPISendData();
    
    
};

#endif

