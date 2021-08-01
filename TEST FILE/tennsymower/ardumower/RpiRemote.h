#include <arduino.h>
#ifndef RpiRemote_h
#define RpiRemote_h
//#define RaspberryPIPort SerialUSB
class Robot;

class RpiRemote
{
  public:
    void setRobot(Robot *aRobot);
    class Tokeniser
    {
      public:
        Tokeniser(char* str, char token);
        boolean next(char* out, int len);

      private:

        char* str;
        char token;
    };



    uint8_t create_checksum(String lineOfString);
    void init();
    void run();
    void readPi();
    void writePi(String stringLine);
    void sendCommandToPi(String stringLine);
    void SendStatusToPi();
    void SendRfidToPi();
    

  private:
    unsigned long nextTimeRaspberryPISendStat; // delay between 2 data send to RPI for stat
    unsigned long nextTimeRaspberryPISendInfo; // delay between 2 data send to RPI for info
    unsigned long nextTimeRaspberryPISendMot; // delay between 2 data send to RPI for wheel motor
    unsigned long nextTimeRaspberryPISendMow; // delay between 2 data send to RPI for mow motor
    unsigned long nextTimeRaspberryPISendPeri; // delay between 2 data send to RPI for perimeter
    unsigned long nextTimeRaspberryPISendBat; // delay between 2 data send to RPI for battery
    unsigned long nextTimeRaspberryPISendByLane; // delay between 2 data send to RPI for bylane
    unsigned long nextTimeRaspberryPISendImu; // delay between 2 data send to RPI for Imu




    int delayInfoToPi ;  //duration between 2 answers
    int maxRepetInfoToPi ;  // max number of answer
    int delayMotToPi ;  //duration between 2 answers
    int maxRepetMotToPi ;  // max number of answer
    int delayMowToPi ;  //duration between 2 answers
    int maxRepetMowToPi ;  // max number of answer
    int delayPeriToPi ;  //duration between 2 answers
    int maxRepetPeriToPi ;  // max number of answer
    int delayBatToPi ;  //duration between 2 answers
    int maxRepetBatToPi ;  // max number of answer
    int delayByLaneToPi ;  //duration between 2 answers
    int maxRepetByLaneToPi ;  // max number of answer
    int delayImuToPi ;  //duration between 2 answers
    int maxRepetImuToPi ;  // max number of answer
    
    boolean check_checksum();
    Robot *robot;
    uint8_t parse_hex(char h);
    boolean encode(char c);
    char buf[120];
    uint8_t pos;
    boolean process_buf();
    void read_pfo();
    void readWrite_setting();
    void readWrite_var();
    void receive_command();
    void receive_request();
    void RaspberryPISendStat ();
   
    
    void receivePiCommand (String ActuatorName, int value);
    void receivePiReqSetting (String Setting_page, int nb_page);
    void RaspberryPISendDebug (String data);
    void RaspberryPISendInfo ();
    void RaspberryPISendMot ();
    void RaspberryPISendMow ();
    void RaspberryPISendPeri ();
    void RaspberryPISendBat ();
    void RaspberryPISendByLane ();
    void RaspberryPISendImu ();
    
};

#endif
