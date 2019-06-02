/**
   Multi-PN5180
   An "escape room"-style puzzle in which a number of RFID tags must be placed in front
   of the correct PN5180 RFID readers in order to be detected and solve the puzzle.
*/
// ESP-32    <--> PN5180 pin mapping:
// Vin       <--> 5V  if esp32 powered using USB
// 3.3V      <--> 3.3V
// GND       <--> GND
// SCLK, 18   --> SCLK
// MISO, 19  <--  MISO
// MOSI, 23   --> MOSI
// SS, 12     --> NSS (=Not SS -> active LOW)
// BUSY, 13   <--  BUSY
// Reset, 14  --> RST
//
// DEFINES
// #define DEBUG

// INCLUDES
// Download from https://github.com/playfultechnology/PN5180-Library
#include <PN5180.h>
#include <PN5180ISO15693.h>

// GLOBALS
// Each PN5180 reader requires unique NSS, BUSY, and RESET pins,
// as defined in the constructor below
PN5180ISO15693 nfc(12, 13, 14);

// Array to record the value of the last UID read by each reader
uint8_t lastUid[8];
unsigned long lastPrintWatchDog;

void setup() {


  // Initialise serial connection
  Serial.begin(115200);
  
 


  Serial.println(F("Initialising..."));
  nfc.begin();
  Serial.println(F("Resetting..."));
  nfc.reset();
  Serial.println(F("Enabling RF field..."));
  nfc.setupRF();

  Serial.println(F("Setup Complete"));

  
}

void loop() {
 
  if (millis() > lastPrintWatchDog) {
    lastPrintWatchDog = millis() + 60000;
    Serial.print("WD");
    Serial.println(millis() / 1000);
  }
  // Variable to store the ID of any tag read by this reader
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
      //Serial.print(F("New Card Detected"));
      //Serial.print(F("... "));
      String mastring=String((char *)thisUid);
      for (int j = 0; j < sizeof(thisUid); j++) {
        Serial.print(thisUid[j], HEX);
      }
      
      Serial.println();
      // Update the array that keeps track of most recent ID
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
      // Update the array that keeps track of last known ID
      memset(lastUid, 0, sizeof(lastUid[0]) * 8);
    }

#ifdef DEBUG
    Serial.print(F("Error when reading : "));
    Serial.println(nfc.strerror(rc));
#endif
  }



}
