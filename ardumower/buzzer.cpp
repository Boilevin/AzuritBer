#include "buzzer.h"
#include "config.h"
#include <Arduino.h>
#include "DueTimer.h"
#define pinBuzzer 53 
BuzzerClass Buzzer;

static boolean tone_pin_state = false;

void toneHandler() {
  digitalWrite(pinBuzzer, tone_pin_state = !tone_pin_state);
}

void BuzzerClass::begin()
{
  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, LOW);
}

void BuzzerClass::tone( unsigned int  freq )
{
  pinMode(pinBuzzer, OUTPUT);
  Timer1.attachInterrupt(toneHandler).setFrequency(freq).start();

}

void BuzzerClass::noTone() {
  Timer1.stop();
  digitalWrite(pinBuzzer, LOW);
}



