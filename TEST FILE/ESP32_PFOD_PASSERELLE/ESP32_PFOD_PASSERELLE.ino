#include <PN5180.h>
#include <PN5180ISO15693.h>
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
BluetoothSerial ESP_BT; //Object for Bluetooth
unsigned long nextSend;
PN5180ISO15693 nfc(12, 13, 14);
uint8_t lastUid[8];
//unsigned long lastPrintWatchDog;

void setup() {
  Serial2.begin(115200);

  ESP_BT.begin("ESP32_BT");
  nextSend = 1000;

  nfc.begin(); 

  nfc.reset();

  //nfc.setupRF();



}

void loop() {

  while (ESP_BT.available()) {
    Serial2.write(ESP_BT.read());
  }
  while (Serial2.available()) {
    ESP_BT.write(Serial2.read());
  }

  if (millis() > nextSend) {
    nextSend = millis() + 1000;
    Serial2.println("{RFID123456789}");
  }
  delay(1);
}
