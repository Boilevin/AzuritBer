#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <detail/RequestHandlersImpl.h>
#include <Ticker.h>
#include <SPIFFS.h>
#include <esp_wifi.h>

//bber1
//*********RFID**************//
#include "PN5180.h"
#include "PN5180ISO15693.h"
#define rfid_board_IsPluged false //Set to true or false according your hardware
#define debug_rfid true

//bber2
//*********MQTT*************//
#include "PubSubClient.h"
boolean useMqtt = true ; //Set to true only if you have a mosquito broker running in your house or false
const char* mower_name = "RL1000";//your mower name : help if you have many mower connected to the same broker
const char* mqtt_server = "10.0.0.8";
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "admin";
const char* mqtt_pass = "admin";
const char* mqtt_subscribeTopic1 = "/COMMAND/#";
// actual possible received topic from hassio : "START" "STOP" "HOME" "STARTTIMER")
const char* mqtt_tempTopic = "/Temp";
const char* mqtt_batteryTopic = "/Battery";
const char* mqtt_idleTopic = "/Idle";
const char* mqtt_statusTopic = "/Status";
const char* mqtt_stateTopic = "/State";
const char* mqtt_id = "Mower";//do not change
#define debug_mqtt true

#define LED 2


#define SERIAL_BUFFER_SIZE 256

#define MAX_CONFIG_LEN  100
#define MSG_HEADER "[WSB]"
#define VERSION " Version vom 29.04.2022 experimentell wegen LiveData-Funktion!!!"
#define CONFIG_MSG_START "config:"

#define SET_IP_SETTING  1  //true=IP-Settings from azurit-programm, false=IP-Settings by user/esp


// if you use Serial for esp to pcb communication and at the same time for serial connection you must keep in mind:
// -Writing on USB/serial from pc to esp will not be possible, you have to cut the connection to the pcb
// -Oberserving the esp work with arduino-Monitor will only read data, so communikation between pcb and esp still works

//select your connections
//#define Serial_ESP_to_PCB  Serial    // Serial Connection between ESP and PCB, ESP-Pin : RX0/TX0
//#define Serial_ESP_to_PCB  Serial1 // Serial Connection between ESP and PCB, ESP-Pin : RX1/TX1
#define Serial_ESP_to_PCB  Serial2 // Serial Connection between ESP and PCB, ESP-Pin : RX2/TX2
#define Serial_ESP_to_USB  Serial    // Serial Connection from ESP via USB to PC (arduino Monitor/Plotter)
#define BAUDRATE_ESP_to_PCB 115200
#define BAUDRATE_ESP_to_USB 115200

IPAddress myIP(10, 0, 0, 168);
IPAddress network_gateway(10, 0, 0, 1);
IPAddress network_subnet(255, 255, 255, 0);
IPAddress network_dns(10, 0, 0, 1);
char *wifi_network_ssid     = "your ssid";
char *wifi_network_password = "password";

bool wifiConnected = false;
int connectCnt = 0;
WebServer server(80); // Webserver-Instanz für Port 80 erstellen
WiFiServer PFODserver(8080); // TCPserver-Instanz für PFOD-App auf Port 8080 erstellen

WiFiClient PFODclient;
bool PFODclientConnected = false;

//bber2
WiFiClient mqttClient;
PubSubClient client(mqttClient);
char line_receive[256];
unsigned long next_mqtt_test_connection;


#define MENU    0
#define LINK    1
#define SLIDER  2
#define PLOT    3

#define DEBUG 0

#define PARAMID_SSID    0
#define PARAMID_PASSWD  1
#define PARAMID_LOCALIP 2
#define PARAMID_GATEWAY 3
#define PARAMID_SUBNET  4
#define NBPARAMS (sizeof(params)/sizeof(params[0]))

/*String msgTest1 = "{.Battery`1000|j00~Battery 29.37 V|j01~Monitor YES|j02~Go home if below Volt `237`293`211~ ~0.1|j12~Switch off if idle minutes `0`300`1~ ~1|j03~Switch off if below Volt `217`293`211~ ~0.1|j04~Charge 0.06V 0.00A|j08~Charge factor `16`9`59~ ~0.001|j10~charging starts if Voltage is below `320`293`0~ ~0.1|j11~Battery is fully charged if current is below `-999990`16`0~ ~0.1}";
  String msgTest2 = "{.Settings|sz~Save settings|s1~Motor|s2~Mow|s3~BumperDuino|s4~Sonar|s5~Perimeter|s6~Lawn sensor|s7~IMU|s8~R/C|s9~Battery|s10~Station|s11~Odometry|s13~Rain|s15~Drop sensor|s14~GPS|i~Timer|s12~Date/time|sx~Factory settings}";
  String msgTest3 = "{.Motor`1000|a00~Overload Counter l, r 0, 0|a01~Power in Watt l, r 0.00, 0.00|a05~motor current in mA l, r 0.00, 0.00|a02~Power max `750`1000`0~ ~0.1|a03~calibrate left motor  `0`1000`0~ ~1|a04~calibrate right motor `0`1000`0~ ~1|a05~Speed l, r pwm0.00, 0.00|a15~Speed max in pwm `255`255`0~ ~1|a11~Accel `1000`2000`500~ ~1|a18~Power ignore time `2000`8000`0~ ~1|a07~Roll time max `1500`8000`0~ ~1|a19~Roll time min `750`1000`0~ ~1|a08~Reverse time `1200`8000`0~ ~1|a09~Forw time max `12000`8000`0~ ~10|a12~Bidir speed ratio 1 `30`100`0~ ~0.01|a13~Bidir speed ratio 2 `92`100`0~ ~0.01|a10~Testing isOFF|a14~for config file:motorSenseScale l, r6.15, 6.15|a16~Swap left direction NO|a17~Swap right direction NO}";
  String msgTest4 = "{.Sonar`1000|d00~Use YES|d04~Use left YES|d05~Use center YES|d06~Use right YES|d01~Counter 0|d02~Value l, c, r 95, 105, 0|d03~Trigger below (0=off) `1797`3000`0~ ~1|d07~Slow below `1115`3000`0~ ~1}";
*/
//bber1
//if (rfid_board_IsPluged) {
PN5180ISO15693 nfc(12, 13, 14);
//}

uint8_t lastUid[8];
String SplitResult[10];

typedef struct {
  const char* name;
  const char* valueStr;
} param_t;

param_t params[] = {
  {"SSID", ""},
  {"Password", ""},
  {"IPAddress", ""},
  {"Gateway", ""},
  {"Subnet", ""},
};

struct PFODelement {
  String cmd;
  String title;
  String value;
  String sl_min;
  String sl_max;
  String sl_res;
  int type;
};

typedef struct ledSequence_t {
  uint8_t onTicks;
  uint8_t offTicks;
} ledSequence_t;

const ledSequence_t ledSeq_startup        =   {1, 10};
const ledSequence_t ledSeq_waitForConfig  =   {1, 1};
const ledSequence_t ledSeq_connecting  =      {3, 3};
const ledSequence_t ledSeq_connected  =       {1, 0};
const ledSequence_t ledSeq_clientConnected  = {10, 1};
Ticker ledTicker;

PFODelement elemente[30];
int nElemente = 0;

char* SerialMsg = "";
String lastMSG = "";
long timer1s = 0UL;
long counter = 0UL;

void setup() {
  // Configure Serial Port
  Serial_ESP_to_USB.begin(BAUDRATE_ESP_to_USB);
  Serial_ESP_to_USB.setTimeout(500);
  Serial_ESP_to_PCB.begin(BAUDRATE_ESP_to_PCB);
  Serial_ESP_to_PCB.setTimeout(500);

  // Configure LED
  pinMode(LED, OUTPUT);
  setLedSequence(ledSeq_startup);
  ledTicker.attach(0.1, onLedTicker);

  // Welcome message
  delay(500);
  Serial_ESP_to_USB.println("\n\n");
  Serial_ESP_to_USB.println(MSG_HEADER" ESP32 Serial_ESP_to_USB WIFI Bridge with Webinterface for AzuritBer on Teensy4.1 ! (not Azurit on Due/Mega)");
  Serial_ESP_to_USB.println(MSG_HEADER" Developed by great software engineers, only slightly modified by Prince Ruprecht");
  Serial_ESP_to_USB.println(MSG_HEADER""VERSION);

  connectWIFI();

  setLedSequence(ledSeq_connecting);

  if (!SPIFFS.begin()) {
    debugln("SPIFFS not initialized! Stop!");
    while (1) yield();
  }

  // add handler for webserver
  server.onNotFound([]() {
    if (!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });
  // Die Abfrage auf die reine URL '/' wird auf '/index.html' umgelenkt
  server.serveStatic("/", SPIFFS, "/index.html");
  // Files ausliefern
  server.on("/", HTTP_GET, handleRoot);
  // LED steuern
  server.on("/set", HTTP_POST, handleSet);
  // Werte abrufen
  server.on("/werte", HTTP_POST, handleWerte);

  server.begin(); // Web-Server starten
  debugln("HTTP Server running on Port 80");

  flushInput();

  // Start PFODserver
  PFODserver.begin();
  PFODserver.setNoDelay(true);


  //bber1
  //********************************RFID code*************************************
  if (rfid_board_IsPluged) {
    // rfid reader init
    if (debug_rfid) Serial_ESP_to_USB.println("Begin the RFID");
    nfc.begin();
    if (debug_rfid) Serial_ESP_to_USB.println("Reset the RFID");
    nfc.reset();
    if (debug_rfid) Serial_ESP_to_USB.println("Init the RFID");
    nfc.setupRF();
    if (debug_rfid) Serial_ESP_to_USB.println("RFID READER READY");
  }
  else
  {
    if (debug_rfid) Serial_ESP_to_USB.println("RFID READER NOT USE");
  }
  //bber2
   //********************************MQTT code*************************************
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(receivedCallback);
}

void loop() {

  Check_WIFI();
  server.handleClient();  //Webserver
  handlePFODclient();
  handleSerialInput();

  if (millis() > timer1s + 1000UL) {
    timer1s = millis();
  }
  //bber1
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

        Serial_ESP_to_PCB.print("{RFID"); // pfod start message with {

        for (int j = 0; j <= 2; j++) {
          Serial_ESP_to_PCB.print(lastUid[j], HEX);
        }
        Serial_ESP_to_PCB.println("}"); //pfod stop message with }
        // Update the array that keeps track of most recent ID
        // Update the array that keeps track of last known ID
        memset(lastUid, 0, sizeof(lastUid[0]) * 8);
      }

    }
  }
//bber2
//*********MQTT*************//
  if ((useMqtt) && (!client.connected()) && (millis() > next_mqtt_test_connection))
  {
    next_mqtt_test_connection = millis() + 5000;
    mqttconnect();
  }
}

void handleSerialInput() {
  if (PFODclientConnected) {
    if (Serial_ESP_to_PCB.available()) {
      size_t len = min(Serial_ESP_to_PCB.available(), 255);
      char sbuf[len];
      String erg = "";
      Serial_ESP_to_PCB.readBytes(sbuf, len);
      PFODclient.write(sbuf, len);

    }
  }
}

void debug(String msg) {
  if (DEBUG) {
    Serial_ESP_to_USB.print(MSG_HEADER);
    Serial_ESP_to_USB.print(" ");
    Serial_ESP_to_USB.print(msg);
  }
}

void debugln(String msg) {
  if (DEBUG) {
    Serial_ESP_to_USB.print(MSG_HEADER);
    Serial_ESP_to_USB.print(" ");
    Serial_ESP_to_USB.println(msg);
  }
}

void flushInput(void) {
  while (Serial_ESP_to_PCB.available())
    Serial_ESP_to_PCB.read();
}

//bber2
//*********MQTT*************//
void receivedCallback(char* topic, byte* payload, unsigned int payload_length) { //data coming from mqtt

  //convert payload to string
  String payloadString = "";
  for (int i = 0; i < payload_length; i++) {
    payloadString = payloadString + String(((char)payload[i]));
  }
  if (debug_mqtt)  Serial.print("Mqtt topic received: ");
  if (debug_mqtt)  Serial.println(topic);
  if (debug_mqtt)  Serial.print("payload: ");
  if (debug_mqtt)  Serial.println(payloadString);

  if (payloadString.length() > 0) {
    //split payload
    int count = csvSplit(payloadString);
    for (int j = 0; j < count; ++j)
    {
      if (SplitResult[j].length() > 0) {
        if (debug_mqtt) Serial.print(j);
        if (debug_mqtt) Serial.print(" ");
        if (debug_mqtt) Serial.println(SplitResult[j]);
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
void mqttconnect() {

  if (!client.connected()) {
    if (debug_mqtt) Serial.print("MQTT connecting ...");

    if (client.connect(mqtt_id, mqtt_user, mqtt_pass)) {
      if (debug_mqtt) Serial.println("connected");
      char outMessage[strlen(mower_name) + strlen(mqtt_subscribeTopic1)];
      sprintf(outMessage, "%s%s", mower_name, mqtt_subscribeTopic1);
      if (debug_mqtt) Serial.print("Subscribe to : ");
      if (debug_mqtt) Serial.println(outMessage);
      client.subscribe(outMessage);

    } else {
      if (debug_mqtt) Serial.print("mqtt failed, status code =");
      if (debug_mqtt) Serial.print(client.state());
      if (debug_mqtt) Serial.println("try again in 5 seconds");

    }
  }
}
