


#include "Screen.h"
#include "mower.h"
#include "robot.h"





#include "ACROBOTIC_SSD1306.h"
/*
  Screen::Screen()
  {

  }
*/
/*
  void Screen::setRobot(Robot *aRobot) {
  this->robot = aRobot;
  }
*/
void Screen::init() {
  Console.println(" --- Start SCREEN Connection --- ");
  oled.init();    // Initialze SSD1306 OLED display
  delay(500);
  oled.clearDisplay();              // Clear screen
  delay(500);
  oled.setTextXY(2, 2);
  oled.putString("AZURITBER");
  //oled.setTextXY(4, 0);
  //oled.putString(VER);

}
void Screen::refreshMowScreen() {
  if(robot.stateTime<500){
    oled.clearDisplay();   
  }
  else{
    oled.setTextXY(3, 7);
    oled.putFloat(int(robot.batVoltage), 0);
  }
    
 


}
void Screen::refreshStationScreen() {
  oled.setTextXY(2, 7);
  oled.putFloat(int(robot.batVoltage), 0);
}

void Screen::refreshWaitScreen() {

  if (lastScreenbatVoltage != int(robot.batVoltage)) {
    oled.setTextXY(2, 1);
    oled.putString("Bat : ");
    oled.setTextXY(2, 7);
    oled.putString("    ");
    oled.setTextXY(2, 7);
    oled.putFloat(int(robot.batVoltage), 0);
    lastScreenbatVoltage = int(robot.batVoltage);
    Serial.println(lastScreenbatVoltage);

    Serial.println(robot.stateName());
  }
  oled.setTextXY(1, 1);
  oled.putString("loops : ");
  oled.setTextXY(1, 11);
  //oled.putString("      ");
  oled.putFloat(robot.loopsPerSec, 0);
  oled.setTextXY(5, 1);
  oled.putString(robot.statusName());
  oled.setTextXY(6, 1);
  oled.putString(robot.stateName());
}

void Screen::refreshErrorScreen() {

}
void Screen::refreshTrackScreen() {

}

void Screen::erase() {
  oled.clearDisplay();
 // delay(500);
}
