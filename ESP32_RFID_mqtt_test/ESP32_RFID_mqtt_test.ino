//SEE config.h for all the setting
//WARNING to compile without error into tool set the Partition scheme to Huge APP

//Hardware wiring :

//TO RFID BOARD
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

//TO PCB1.3 if used :
// ESP-32    <--> pcb1.3 pin mapping:
// Vin       <--> 5v ON BT CONNECTOR
// GND       <--> GND ON BT CONNECTOR
// RX2       <--> TX ON BT CONNECTOR
// TX2       <--> RX ON BT CONNECTOR

#include "PN5180.h"
#include "PN5180ISO15693.h"

#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "PubSubClient.h"


#include <BluetoothSerial.h>
BluetoothSerial SerialBT;


#include <WiFiClient.h>
WiFiClient pfodClient;
WiFiClient mqttClient;
PubSubClient client(mqttClient);
WiFiServer TheServeur(8881);

uint8_t BTbuf[my_bufferSize];
uint16_t iBT = 0;
uint8_t WIFIbuf[my_bufferSize];
uint16_t inWiFI = 0;
int reconnect_count = 0;

char msg[20];
unsigned long next_test_connection;
long lastMsg = 0;

char line_receive[256];
byte mon_index = 0;
String entete = "";

PN5180ISO15693 nfc(12, 13, 14);
uint8_t lastUid[8];
String SplitResult[10];

int csvSplit(String string)//use to cut a string separator eg "STARTTIMER;1;1;0;25;2;50";
{
  String tempString = "";
  int bufferIndex = 0;
  for (int i = 0; i < string.length(); ++i)
  {
    char c = string[i];
    if (c != ';')
    {
      tempString += c;
    }
    else
    {
      tempString += '\0';
      SplitResult[bufferIndex++] = tempString;
      tempString = "";
    }
  }
  SplitResult[bufferIndex++] = tempString;//ecriture du dernier
  return bufferIndex;
}

void receivedCallback(char* topic, byte* payload, unsigned int payload_length) { //data coming from mqtt

  //convert payload to string
  String payloadString = "";
  for (int i = 0; i < payload_length; i++) {
    payloadString = payloadString + String(((char)payload[i]));
  }
  if (debug)  Serial.print("Mqtt topic received: ");
  if (debug)  Serial.println(topic);
  if (debug)  Serial.print("payload: ");
  if (debug)  Serial.println(payloadString);

  if (payloadString.length() > 0) {
    //split payload
    int count = csvSplit(payloadString);
    for (int j = 0; j < count; ++j)
    {
      if (SplitResult[j].length() > 0) {
        if (debug) Serial.print(j);
        if (debug) Serial.print(" ");
        if (debug) Serial.println(SplitResult[j]);
      }
    }
  }
  if (SplitResult[0] == "START") {
    Serial2.println("{ra}");
  }
  if (SplitResult[0] == "STOP") {
    Serial2.println("{ro}");
  }
  if (SplitResult[0] == "HOME") {
    Serial2.println("{rh}");
  }
  if (SplitResult[0] == "STARTTIMER") {
    Serial2.println("{ya0`" + SplitResult[1] + "}");//F("mowPatternCurr")
    Serial2.println("{ya1`" + SplitResult[2] + "}");//F("laneUseNr")
    Serial2.println("{ya2`" + SplitResult[3] + "}");//F("rollDir")
    Serial2.println("{ya3`" + SplitResult[4] + "}");//F("whereToStart")
    Serial2.println("{ya4`" + SplitResult[5] + "}");//F("areaToGo")
    Serial2.println("{ya5`" + SplitResult[6] + "}");//F("actualLenghtByLane")
    Serial2.println("{rv}");
  }
}

void mqttconnect() {

  if (!client.connected()) {
    if (debug) Serial.print("MQTT connecting ...");

    if (client.connect(mqtt_id, mqtt_user, mqtt_pass)) {
      if (debug) Serial.println("connected");
      //const char* cmd_msg = "/COMMAND/#";
      char outMessage[strlen(mower_name) + strlen(mqtt_subscribeTopic1)];
      sprintf(outMessage, "%s%s", mower_name, mqtt_subscribeTopic1);
      if (debug) Serial.print("Subscribe to : ");
      if (debug) Serial.println(outMessage);
      client.subscribe(outMessage);

    } else {
      if (debug) Serial.print("mqtt failed, status code =");
      if (debug) Serial.print(client.state());
      if (debug) Serial.println("try again in 5 seconds");

    }
  }
}

void start_stop_AreaSender() {
  //line_receive is a char array separate by ,
  //example to stop sender on ip 15  "#SENDER,10.0.0.15,A0
  //example to start sender on ip 15  "#SENDER,10.0.0.15,A1
  HTTPClient http;
  char val1[10], val2[20], val3[10];
  sscanf(line_receive, "%[^,],%[^,],%[^,]", val1, val2, val3);
  String serverPath = "http://" + String(val2) + "/" + String(val3);
  http.begin(serverPath.c_str());
  http.GET();
}

void esp32_Mqtt_sta() {
  //receive state from mower msgid,status,state,temp,battery,idle
  //message separation
  char val1[6], val2[20], val3[20], val4[20], val5[20], val6[20];
  sscanf(line_receive, "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,]", val1, val2, val3, val4, val5, val6);

  //status
  char outTopic1[strlen(mower_name) + strlen(mqtt_statusTopic)];
  sprintf(outTopic1, "%s%s", mower_name, mqtt_statusTopic);
  client.publish(outTopic1, val2);
  //state
  char outTopic2[strlen(mower_name) + strlen(mqtt_stateTopic)];
  sprintf(outTopic2, "%s%s", mower_name, mqtt_stateTopic);
  client.publish(outTopic2, val3);
  //temp
  char outTopic3[strlen(mower_name) + strlen(mqtt_tempTopic)];
  sprintf(outTopic3, "%s%s", mower_name, mqtt_tempTopic);
  client.publish(outTopic3, val4);
  //battery
  char outTopic4[strlen(mower_name) + strlen(mqtt_batteryTopic)];
  sprintf(outTopic4, "%s%s", mower_name, mqtt_batteryTopic);
  client.publish(outTopic4, val5);
  //idle
  char outTopic5[strlen(mower_name) + strlen(mqtt_idleTopic)];
  sprintf(outTopic5, "%s%s", mower_name, mqtt_idleTopic);
  client.publish(outTopic5, val6);
}



void setup() {
  delay(500);
  Serial.begin(115200);
  Serial2.begin(19200);
  if (debug) Serial.println("********* ESP32 BT and WiFi serial bridge ******************");

  //********************************WIFI init code*************************************

  if (MODE_STA == true) {
    if (debug) Serial.println("Start ESP32 Station mode");
    reconnect_count = 0;
    WiFi.mode(WIFI_STA);
    WiFi.config(ip, gateway, netmask);
    WiFi.begin(ssid, pw);
    if (debug) Serial.print("Try to Connect to your Wireless network: ");

    while ((WiFi.status() != WL_CONNECTED) ) {
      delay(500);
      if (debug) Serial.print(".");
      reconnect_count = reconnect_count + 1;
      if (reconnect_count > 10) {
        
        Serial2.end();
        delay(500);
        ESP.restart();
        
        break;
      }
    }
    if ((debug) && (MODE_AP != true)) {
      Serial.println(" ");
      Serial.print("WiFi connected ");
      Serial.print(WiFi.SSID());
      Serial.print(" IP= ");
      Serial.println(ip);
      Serial.println(" ");

    }
  }
  if (MODE_AP == true) {
    useMqtt = false;
    if (debug) Serial.println("Your ESP32 is the Access Point");
    //AP mode (phone connects directly to ESP) (no router)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid_ap); // configure ssid and password for softAP
    delay(2000); // VERY IMPORTANT
    WiFi.softAPConfig(ip_ap, ip_ap, netmask_ap); // configure ip address for softAP
    if (debug) {
      Serial.print("ESP Access Point mode started SSID : ");
      Serial.print(ssid_ap);
      Serial.print(" IP= ");
      Serial.println(ip_ap);
    }
  }

  if (debug) Serial.println("Starting WIFI Serveur on port 8881");
  TheServeur.begin(); // start TCP server
  TheServeur.setNoDelay(true);

  //********************************BT code*************************************

  if (debug) {
    Serial.println("");
    Serial.println("Start Bluetooth Server");
    Serial.print("BT Name : ");
    Serial.println(mower_name);
    Serial.println("");
  }
  SerialBT.begin(mower_name); //Bluetooth device name


  //********************************MQTT code*************************************
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(receivedCallback);

  //********************************RFID code*************************************
  if (rfid_board_IsPluged) {
    // rfid reader init
    if (debug) Serial.println("Begin the RFID");
    nfc.begin();
    if (debug) Serial.println("Reset the RFID");
    nfc.reset();
    if (debug) Serial.println("Init the RFID");
    nfc.setupRF();
    if (debug) Serial.println("RFID READER READY");
  }
  else
  {
    if (debug) Serial.println("RFID READER NOT USE");
  }

}

void loop() {
  //********************************MQTT code*************************************
  if ((useMqtt) && (!client.connected()) && (millis() > next_test_connection))
  {
    next_test_connection = millis() + 5000;
    mqttconnect();
  }
  //********************************RFID code*************************************
  if (rfid_board_IsPluged)
  {
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
        memcpy(lastUid, thisUid, sizeof(lastUid[0]) * 8);
      }
    }
    // If a card cannot be read
    else {
      // Test if we previously knew about a card (in which case it's just been removed
      // The most significant (last) byte of a valid UID should always be 0xE0. e.g. E007C4A509C247A8
      if (lastUid[7] == 0xE0) {

        Serial2.print("{RFID"); // pfod start message with {

        for (int j = 0; j <= 2; j++) {
          Serial2.print(lastUid[j], HEX);
        }
        Serial2.println("}"); //pfod stop message with }
        // Update the array that keeps track of most recent ID
        // Update the array that keeps track of last known ID
        memset(lastUid, 0, sizeof(lastUid[0]) * 8);
      }

    }
  }


  // receive from Bluetooth:
  if (SerialBT.hasClient())
  {
    while (SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from BT client
      if (iBT < my_bufferSize - 1) iBT++;
    }
    Serial2.write(BTbuf, iBT); // now send to serial2:
    iBT = 0;
  }


  if (TheServeur.hasClient())
  {
    //find free/disconnected spot
    if (!pfodClient || !pfodClient.connected()) {
      if (pfodClient) pfodClient.stop();
      pfodClient = TheServeur.available();
      if (debug) Serial.println("New WIFI client ");
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = TheServeur.available();
    TmpserverClient.stop();
  }


  //data coming from pfod
  if (Serial2 != NULL)
  {
    if (pfodClient)
    {
      while (pfodClient.available())
      {
        WIFIbuf[inWiFI] = pfodClient.read(); // read char from client
        if (inWiFI < my_bufferSize - 1) inWiFI++;
      }
      Serial2.write(WIFIbuf, inWiFI); // now send to UART(2):
      inWiFI = 0;
    }
    if (Serial2.available())
    {
      while (Serial2.available())
      {
        WIFIbuf[inWiFI] = Serial2.read(); // read char from UART(2)
        char aChar = WIFIbuf[inWiFI];
        if (inWiFI < my_bufferSize - 1) inWiFI++;

        if (aChar == '\n')
        {
          // End of record detected. Time to parse and check for non pfod sentence
          //here data coming from mqtt or pfod over wifi
          if (debug) Serial.println(line_receive);
          if (strncmp(line_receive, "$SENDER", 7) == 0) {
            start_stop_AreaSender();
          }
          if (strncmp(line_receive, "#RMSTA", 6) == 0) {
            esp32_Mqtt_sta();
          }

          mon_index = 0;
          line_receive[mon_index] = NULL;
        }
        else
        {
          line_receive[mon_index] = aChar;
          mon_index++;
          line_receive[mon_index] = '\0'; // Keep the string NULL terminated
        }
      }
      if (pfodClient)
        pfodClient.write(WIFIbuf, inWiFI);
      // now send to Bluetooth:
      if (SerialBT.hasClient())
        SerialBT.write(WIFIbuf, inWiFI);

      inWiFI = 0;
    }

  }

  client.loop();

}
