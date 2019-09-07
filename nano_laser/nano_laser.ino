#include "Wire.h"
#include "VL53L0X.h"
#include "RpiRemote.h"

#define XSHUT_pin6 10
#define XSHUT_pin5 9
#define XSHUT_pin4 8
#define XSHUT_pin3 7
#define XSHUT_pin2 6
#define XSHUT_pin1 5

//ADDRESS_DEFAULT 0b0101001 or 41
#define Sensor1_newAddress 41 //not required address change
#define Sensor2_newAddress 42
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44
#define Sensor5_newAddress 45
#define Sensor6_newAddress 46

//Setting for Raspberry -----------------------------------
RpiRemote MyRpi;

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;
VL53L0X Sensor5;
VL53L0X Sensor6;

int rainPin = 12;
int speed_factor=10; // millimeter mower move between 2 reading
bool rain_detect;
long val1;
long val2;
long val3;
//need to store the last read to check that the next read is inferior because mower run forward
int sonar1_last_read;
int sonar2_last_read;
int sonar3_last_read;
//store the number of correct following reading (reset when the read is > at the last_read
byte sonar1_qty_read;
byte sonar2_qty_read;
byte sonar3_qty_read;
//On when 5 read succesfull in 1 sec
bool sonar1_trigged;
bool sonar2_trigged;
bool sonar3_trigged;

unsigned long last_millis_sonar1;
unsigned long last_millis_sonar2;
unsigned long last_millis_sonar3;

//#define Serial_BAUDRATE    250000       // baudrate used for Raspberry PI Serial

#define Serial Serial

void setup()
{
  MyRpi.init();
  sonar1_last_read = 1000;
  sonar2_last_read = 1000;
  sonar3_last_read = 1000;

  pinMode(rainPin, INPUT);

  //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);
  pinMode(XSHUT_pin5, OUTPUT);
  pinMode(XSHUT_pin6, OUTPUT);

  //Serial.begin(115200);

  Wire.begin();
  //Change address of sensor and power up next one
  pinMode(XSHUT_pin6, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
  Sensor6.setAddress(Sensor6_newAddress);
  pinMode(XSHUT_pin5, INPUT);
  delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
  Sensor5.setAddress(Sensor5_newAddress);
  pinMode(XSHUT_pin4, INPUT);
  delay(10);
  Sensor4.setAddress(Sensor4_newAddress);
  pinMode(XSHUT_pin3, INPUT);
  delay(10);
  Sensor3.setAddress(Sensor3_newAddress);
  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);
  pinMode(XSHUT_pin1, INPUT);
  delay(10);

  Sensor1.init();
  Sensor2.init();
  Sensor3.init();
  //Sensor4.init();
  //Sensor5.init();
  //Sensor6.init();

  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor3.setTimeout(500);
  //Sensor4.setTimeout(500);
  //Sensor5.setTimeout(500);
  //Sensor6.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  Sensor1.startContinuous();
  Sensor2.startContinuous();
  Sensor3.startContinuous();
  //Sensor4.startContinuous();
  //Sensor5.startContinuous();
  //Sensor6.startContinuous();

}
void SendData()
{
  String lineToSend;
  lineToSend = "RMTOW,";
  lineToSend = lineToSend + sonar1_last_read;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + sonar2_last_read;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + sonar3_last_read;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "1000";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "1000";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "1000";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + !rain_detect;
  lineToSend = lineToSend + ",";
  MyRpi.writePi(lineToSend);
}
void loop()
{
  val1 = Sensor1.readRangeContinuousMillimeters();
  val2 = Sensor2.readRangeContinuousMillimeters();
  val3 = Sensor3.readRangeContinuousMillimeters();

  //management of the sensor1 ------------------------------------------------------------------
  if (val1 < 800) {
    last_millis_sonar1 = millis();
    if (val1 <= sonar1_last_read-speed_factor) {
      sonar1_qty_read = sonar1_qty_read + 1;
    }
    else
    {
      //Serial.println("reset sonar 1");
      sonar1_qty_read = 0;
    }
    sonar1_last_read = val1;
  }
  if (sonar1_qty_read >= 5) {
    SendData();
    //Serial.println("Sonar 1 trig");
    sonar1_trigged = true;
    sonar1_qty_read = 0;
    sonar1_last_read = 1000;
    delay(1000);
  }
  if (millis() >= last_millis_sonar1 + 2000 ) {
    sonar1_trigged = false;
    sonar1_qty_read = 0;
    sonar1_last_read = 1000;
  }

  //management of the sensor2 ------------------------------------------------------------------
  if (val2 < 800) {
    last_millis_sonar2 = millis();
    if (val2 <= sonar2_last_read-speed_factor) {
      sonar2_qty_read = sonar2_qty_read + 1;
    }
    else
    {
      sonar2_qty_read = 0;
    }
    sonar2_last_read = val2;
  }
  if (sonar2_qty_read >= 5) {
    SendData();
    //Serial.println("Sonar 2 trig");
    sonar2_trigged = true;
    sonar2_qty_read = 0;
    sonar2_last_read = 1000;
    delay(1000);
  }
  if (millis() >= last_millis_sonar2 + 2000 ) {
    sonar2_trigged = false;
    sonar2_qty_read = 0;
    sonar2_last_read = 1000;
  }

  //management of the sensor3 ------------------------------------------------------------------
  if (val3 < 800) {
    last_millis_sonar3 = millis();
    if (val3 <= sonar3_last_read-speed_factor) {
      sonar3_qty_read = sonar3_qty_read + 1;
    }
    else
    {
      sonar3_qty_read = 0;
    }
    sonar3_last_read = val3;
  }
  if (sonar3_qty_read >= 5) {
    SendData();
    //Serial.println("Sonar 3 trig");
    sonar3_trigged = true;
    sonar3_qty_read = 0;
    sonar3_last_read = 1000;
    delay(1000);
  }
  if (millis() >= last_millis_sonar3 + 2000 ) {
    sonar3_trigged = false;
    sonar3_qty_read = 0;
    sonar3_last_read = 1000;
  }


  // slow the loop for correct working of the 5 successfull detection.

  rain_detect = digitalRead(rainPin);
  // print out the state of the button:
  if (rain_detect == 0) {
    SendData();
    //Serial.println("il pleut");
  }

  delay(20);
}
