 // --------------------------------------
// i2c_scanner for I2C1 bus on arduino Due
//
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#define TCAADDR 0x70


void setup()
{
  tcaselect(0);
  Wire1.begin();

  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nNative DUE I2C Scanner");
   Wire1.beginTransmission(0x68);
   Wire1.write(0x37);
   Wire1.write(0x02);
   Wire1.endTransmission();
   Wire1.beginTransmission(0x68);
   Wire1.write(0x6A);
   Wire1.write(0x00);
   Wire1.endTransmission();
   Wire1.beginTransmission(0x68);
   Wire1.write(0x6B);
   Wire1.write(0x00);
   Wire1.endTransmission();
  
}


void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire1.beginTransmission(TCAADDR);
  Wire1.write(1 << i);
  Wire1.endTransmission();  
}
void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning bus I2C1...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C1 device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    //  Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C1 devices found\n");
  else
    Serial.println(millis());

  delay(500);           // wait 5 seconds for next scan
}
