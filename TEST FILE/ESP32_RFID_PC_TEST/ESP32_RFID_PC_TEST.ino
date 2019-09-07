// ESP-32    <--> PN5180 pin mapping:
// Vin       <--> 5V  if esp32 powered using USB YOU NEED TO SEND THE 5V TO PN5180
// 3.3V      <--> 3.3V
// GND       <--> GND
// SCLK, GPIO18   --> SCLK
// MISO, GPIO19  <--  MISO
// MOSI, GPIO23   --> MOSI
// SS, GPIO12     --> NSS (=Not SS -> active LOW)
// BUSY, GPIO13   <--  BUSY
// Reset, GPIO14  --> RST
//

// ESP-32    <--> pcb1.3 pin mapping:
// Vin       <--> 5v ON BT CONNECTOR
// GND       <--> GND ON BT CONNECTOR
// RX2       <--> TX ON BT CONNECTOR
// TX2       <--> RX ON BT CONNECTOR


#include "PN5180.h"
#include "PN5180ISO15693.h"
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
BluetoothSerial ESP_BT; //Object for Bluetooth

PN5180ISO15693 nfc(12, 13, 14);
uint8_t lastUid[8];

void setup() {
 //BT seial for Pfod init
  Serial.begin(19200);
  Serial.println("Setup START");
  ESP_BT.begin("ESP32_BT01");
  Serial.println("BLUETOOTH OK");
// rfid reader init
  nfc.begin(); 
  nfc.reset();
  nfc.setupRF();
  Serial.println("RFID OK");
}

void loop() {

 uint8_t thisUid[8];
  // Try to read a tag ID (or "get inventory" in ISO15693-speak)
  ISO15693ErrorCode rc = nfc.getInventory(thisUid);
  // If the result code was that a card had been read
  if (rc == ISO15693_EC_OK) {
    // If this is the same ID as we read last frame
    if (memcmp(thisUid, lastUid, 8) == 0) {
      // Nothing to do - move on to the next reader
    }
    // If it's a different ID
    else {
      /*
      //Serial.print(F("New Card Detected"));
      //Serial.print(F("... "));
      Serial.print("{RFID"); // pfod start message with {
      for (int j = 0; j <=3; j++) {
        Serial.print(thisUid[j], HEX);
      }
      Serial.println("}"); //pfod stop message with }
      // Update the array that keeps track of most recent ID
      */
      memcpy(lastUid, thisUid, sizeof(lastUid[0]) * 8);
    }
  }
  // If a card cannot be read
  else {
    // Test if we previously knew about a card (in which case it's just been removed
    // The most significant (last) byte of a valid UID should always be 0xE0. e.g. E007C4A509C247A8
    if (lastUid[7] == 0xE0) {
      /*
        Serial.print("Card ");
        for (int j=0; j<sizeof(lastUid); j++) {
        Serial.print(lastUid[j], HEX);
        }
        Serial.print(" removed from Reader ");
        Serial.println();
      */
      Serial.print("{RFID"); // pfod start message with {
      /*
      //i dont use all the byte number 
      for (int j = 0; j < sizeof(thisUid); j++) {
        Serial.print(thisUid[8-j], HEX);
      }
      */
      for (int j = 0; j <=3; j++) {
        Serial.print(lastUid[j], HEX);
      }
      Serial.println("}"); //pfod stop message with }
      // Update the array that keeps track of most recent ID


      
      // Update the array that keeps track of last known ID
      memset(lastUid, 0, sizeof(lastUid[0]) * 8);
    }

#ifdef DEBUG
    Serial.print(F("Error when reading : "));
    Serial.println(nfc.strerror(rc));
#endif
  }

// BT passthrou for 
  while (ESP_BT.available()) {
    Serial.write(ESP_BT.read());
  }
  while (Serial.available()) {
    ESP_BT.write(Serial.read());
  }

  
}
