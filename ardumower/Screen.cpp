/*
  Ardumower (www.ardumower.de)

    2. Set I =0.6 * P and D = 0.125 * P

*/

#include "Screen.h"
#include "mower.h"
#include "robot.h"

#include "ACROBOTIC_SSD1306.h"

Screen::Screen()
{

}

void Screen::init() {
  Console.println(" --- Start SCREEN Connection --- ");
  oled.init();    // Initialze SSD1306 OLED display
  delay(500);
  oled.clearDisplay();              // Clear screen
  delay(500);
  oled.setTextXY(2, 0);
  oled.putString("AZURITBER");
  oled.setTextXY(4, 0);
  //oled.putString(VER);

}
void Screen::refreshMowScreen() {
  //oled.setTextXY(2, 6);
  //oled.putString("           ");
  if (lastScreenbatVoltage != int(robot->batVoltage)) {
    oled.setTextXY(0, 0);
    oled.putString("Bat : ");
    oled.setTextXY(0, 7);
    oled.putString("    ");
    oled.putFloat(int(robot->batVoltage), 0);
    lastScreenbatVoltage = int(robot->batVoltage);
  }
  oled.setTextXY(1, 0);
  oled.putString("loops : ");
  oled.setTextXY(1, 10);
  //oled.putString("      ");
  oled.putFloat(robot->loopsPerSec, 0);
  oled.setTextXY(5, 0);
  oled.putString(robot->statusName());
  oled.setTextXY(6, 0);
  oled.putString(robot->stateName());
}
void Screen::refreshStationScreen() {

}

void Screen::refreshWaitScreen() {

}

void Screen::refreshErrorScreen() {

}
void Screen::refreshTrackScreen() {

}

void Screen::erase() {
  oled.clearDisplay();
  delay(500);
}
