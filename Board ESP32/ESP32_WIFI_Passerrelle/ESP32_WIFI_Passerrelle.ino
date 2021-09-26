//new passerelle for ESP32
//Possible use of ESP32 as standalone Access point or station
// See config.h for parameter
#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>

#ifdef BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif


#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiClient TheClient;
WiFiServer TheServeur(8881);
#endif


uint8_t BTbuf[bufferSize];
uint16_t iBT = 0;
uint8_t WIFIbuf[bufferSize];
uint16_t inWiFI = 0;



void setup() {

  delay(500);
  Serial.begin(115200);
  Serial2.begin(19200);
  if (debug) Serial.println("\n\n ESP32 BT and WiFi serial bridge V1.00");


#ifdef MODE_AP
  if (debug) Serial.println("Open ESP Access Point mode");
  //AP mode (phone connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid); // configure ssid and password for softAP
  delay(2000); // VERY IMPORTANT
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP
  if (debug) Serial.println("ESP Access Point mode started at 192.168.4.1 on port 8881");
#endif


#ifdef MODE_STA
  if (debug) Serial.println("Start ESP32 Station mode");
  // STATION mode (ESP connects to router and use config.h IP and pass value)


  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, netmask);
  WiFi.begin(ssid, pw);
  if (debug) Serial.print("try to Connect to your Wireless network: ");
  if (debug) Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (debug) Serial.print(".");
    ///need to count and stop wifi scanning if more than 20 secondes
  }
  if (debug) Serial.println("\nWiFi connected IP fixe see config.h");
  if (debug) Serial.println("\nUse port 8881");
#endif

#ifdef BLUETOOTH
  if (debug) Serial.println("Start Bluetooth Server");
  SerialBT.begin("Ardumower"); //Bluetooth device name
#endif


#ifdef PROTOCOL_TCP

  
  if (debug) Serial.println("Starting Serveur on port 8881");
  TheServeur.begin(); // start TCP server
  TheServeur.setNoDelay(true);
  
#endif

  // esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
}


void loop()
{


#ifdef BLUETOOTH
  // receive from Bluetooth:
  if (SerialBT.hasClient())
  {
    while (SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from BT client
      if (iBT < bufferSize - 1) iBT++;
    }
    Serial2.write(BTbuf, iBT); // now send to serial2:
    iBT = 0;
  }
#endif

#ifdef PROTOCOL_TCP

  if (TheServeur.hasClient())
  {
    //find free/disconnected spot
    if (!TheClient || !TheClient.connected()) {
      if (TheClient) TheClient.stop();
      TheClient = TheServeur.available();
      if (debug) Serial.println("New WIFI client ");
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = TheServeur.available();
    TmpserverClient.stop();
  }

#endif

  if (Serial2 != NULL)
  {
    if (TheClient)
    {
      while (TheClient.available())
      {
        WIFIbuf[inWiFI] = TheClient.read(); // read char from client
        if (inWiFI < bufferSize - 1) inWiFI++;
      }
      Serial2.write(WIFIbuf, inWiFI); // now send to UART(2):
      inWiFI = 0;
    }
    if (Serial2.available())
    {
      while (Serial2.available())
      {
        WIFIbuf[inWiFI] = Serial2.read(); // read char from UART(2)
        if (inWiFI < bufferSize - 1) inWiFI++;
      }
      if (TheClient)
        TheClient.write(WIFIbuf, inWiFI);

#ifdef BLUETOOTH
      // now send to Bluetooth:
      if (SerialBT.hasClient())
        SerialBT.write(WIFIbuf, inWiFI);
#endif
      inWiFI = 0;
    }

  }
}
