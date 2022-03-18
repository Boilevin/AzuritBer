


#include "Screen.h"
#include "mower.h"
#include "robot.h"

#include <Arduino.h>
#include "U8x8lib.h"
#include "U8g2lib.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

unsigned long aStartReadAt;
unsigned long aEndReadAt;
unsigned long aReadDuration;
int batLevel = 0;

// Please UNCOMMENT one of the contructor lines below
// U8x8 Contructor List
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8x8setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
//U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);        // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


void Screen::init() {
  Console.println(" --- Start SCREEN Connection --- ");

  u8x8.begin();

  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(10, 25);
  u8g2.print("AZURITBER");
  u8g2.setCursor(0, 60);
  u8g2.print(VER);
  u8g2.sendBuffer();

}
void Screen::refreshMowScreen() {

  // battery icon
  batLevel = map(int(robot.batVoltage), int(robot.batSwitchOffIfBelow), int(robot.batFull), 0, 25);
  batLevel = constrain(batLevel, 0, 25);
  u8g2.drawFrame(99, 0, 27, 10);
  u8g2.drawBox(101, 2, batLevel, 6);
  u8g2.drawBox(126, 3, 2, 4);

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(60, 10);
  u8g2.print(int(robot.loopsPerSec));
  u8g2.setCursor(0, 10);
  u8g2.print(robot.perimeterMag);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.setCursor(2, 30);
  u8g2.print(robot.mowPatternName());
  
  if (robot.statusCurr == NORMAL_MOWING) {
    u8g2.setCursor(65, 30);
    u8g2.print("Normal");
  }
  if (robot.statusCurr == SPIRALE_MOWING) {
    u8g2.setCursor(65, 30);
    u8g2.print("Spirale");
  }
  if (robot.statusCurr == WIRE_MOWING) {
    u8g2.setCursor(65, 30);
    u8g2.print("Wire");
  }
  u8g2.setCursor(50, 45);
  u8g2.print((robot.imu.ypr.yaw / PI * 180),1);

  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.setCursor(0, 64);
  u8g2.print(robot.stateName());

  u8g2.sendBuffer();
  u8g2.clearBuffer();

}
void Screen::refreshStationScreen() {
  /*

  */
  u8g2.setFont(u8g2_font_ncenB14_tr);
  if (robot.stateName() == "CHARG") {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(10, 20);
    u8g2.print("Bat Volt :");
    u8g2.setCursor(80, 20);
    u8g2.print(robot.batVoltage, 1);
    u8g2.setCursor(10, 32);
    u8g2.print("Chg Volt :");
    u8g2.setCursor(80, 32);
    u8g2.print(robot.chgVoltage, 1);
    u8g2.setCursor(10, 45);
    u8g2.print("Chg Sens :");
    u8g2.setCursor(80, 45);
    u8g2.print(robot.chgCurrent, 1);
    u8g2.setCursor(10, 62);
    u8g2.print("Duration :");
    u8g2.setCursor(90, 62);
    u8g2.print((millis() - robot.stateStartTime) / 60000);

    u8g2.setFont(u8g2_font_ncenB14_tr);



    //battery icon flashing
    batLevel = batLevel + 1;
    if (batLevel >= 25) batLevel = 0;
    u8g2.drawFrame(99, 0, 27, 10);
    u8g2.drawBox(101, 2, batLevel, 6);
    u8g2.drawBox(126, 3, 2, 4);
  }
  else
  {
    batLevel = map(int(robot.batVoltage), int(robot.batSwitchOffIfBelow), int(robot.batFull), 0, 25);
    batLevel = constrain(batLevel, 0, 25);
    u8g2.drawFrame(99, 0, 27, 10);
    u8g2.drawBox(101, 2, batLevel, 6);
    u8g2.drawBox(126, 3, 2, 4);

    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 10);
    u8g2.print(int(robot.loopsPerSec));
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.setCursor(10, 44);
    u8g2.print("STATION");


  }

  u8g2.sendBuffer();
  u8g2.clearBuffer();

}

void Screen::refreshWaitScreen() {
  // battery icon
  batLevel = map(int(robot.batVoltage), int(robot.batSwitchOffIfBelow), int(robot.batFull), 0, 25);
  batLevel = constrain(batLevel, 0, 25);
  u8g2.drawFrame(99, 0, 27, 10);
  u8g2.drawBox(101, 2, batLevel, 6);
  u8g2.drawBox(126, 3, 2, 4);

  //bumper test

  if (robot.readSensor(SEN_BUMPER_LEFT) == 0) u8g2.drawBox(0, 30, 10, 10);
  if (robot.readSensor(SEN_BUMPER_RIGHT) == 0) u8g2.drawBox(117, 30, 10, 10);


  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(60, 10);
  u8g2.print(int(robot.loopsPerSec));

  u8g2.setCursor(0, 10);
  u8g2.print(robot.perimeterMag);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  //u8g2.setCursor(0, 24);
  //u8g2.print(robot.mowPatternName());
  u8g2.setCursor(40, 40);
  u8g2.print((robot.imu.ypr.yaw / PI * 180), 1);

  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.setCursor(5, 64);
  u8g2.print(robot.statusName());
  u8g2.setCursor(70, 64);
  u8g2.print(robot.stateName());

  u8g2.sendBuffer();
  u8g2.clearBuffer();
}

void Screen::refreshErrorScreen() {
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.setCursor(20, 14);
  u8g2.print("ERROR");
  u8g2.sendBuffer();
  u8g2.clearBuffer();
}
void Screen::refreshTrackScreen() {

}
