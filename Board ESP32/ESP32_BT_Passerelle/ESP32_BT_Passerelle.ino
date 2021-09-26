// ESP-32    <--> pcb1.3 pin mapping:
// Vin       <--> 5v ON BT CONNECTOR
// GND       <--> GND ON BT CONNECTOR
// RX2       <--> TX ON BT CONNECTOR
// TX2       <--> RX ON BT CONNECTOR

#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino
BluetoothSerial ESP_BT; //Object for Bluetooth
void setup() {
//BT seial for Pfod init
  Serial.begin(115200);
  Serial.println("Start Bluetooth");
  ESP_BT.begin("NEW_PCB1");
  Serial.println("Bluetooth started with name NEW_PCB  Wait 10 seconde......");
  delay(10000);
  
  Serial.println("Try to connect to DUE");
  Serial2.begin(19200);
  Serial.println("Connected");
  
}
void loop() {
  while (ESP_BT.available()) {
    Serial2.write(ESP_BT.read());
  }
  while (Serial2.available()) {
    ESP_BT.write(Serial2.read());
  }
}
