#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <detail/RequestHandlersImpl.h>
#include <Ticker.h>
#include <SPIFFS.h>
#include <esp_wifi.h>
#include <ArduinoOTA.h>                // OTA Upload via ArduinoIDE

//*********RFID**************//
#include "PN5180.h"
#include "PN5180ISO15693.h"

//******** YOUR WIFI  ************///
IPAddress myIP(10, 0, 0, 123);
IPAddress network_gateway(10, 0, 0, 1);
IPAddress network_subnet(255, 255, 255, 0);
IPAddress network_dns(10, 0, 0, 1);
char *wifi_network_ssid     = "your ssid";
char *wifi_network_password = "your pass";
boolean useMqtt = false ; //Set to true only if you have a mosquito broker running in your house 
#define rfid_board_IsPluged false //Set to true or false according your hardware PN5180 can read RFID card

#define debug_rfid true
#define debug_mqtt true


//*********MQTT*************//
#include "PubSubClient.h"
const char* mower_name = "Rl1000";//your mower name : help if you have many mower connected to the same broker
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
const char* mqtt_id = "Mower";


#define LED 2

#define MAX_CONFIG_LEN  100
#define MSG_HEADER "[WSB]"
#define VERSION "Ber Version vom 07.05.2022 "
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
#define BAUDRATE_ESP_to_PCB 19200
#define BAUDRATE_ESP_to_USB 115200


bool wifiConnected = false;
int connectCnt = 0;
WebServer server(80); // Webserver-Instanz für Port 80 erstellen
WiFiServer PFODserver(8881); // TCPserver-Instanz für PFOD-App auf Port 8080 erstellen

WiFiClient PFODclient;
bool PFODclientConnected = false;

//bber2
WiFiClient mqttClient;
PubSubClient client_mqtt(mqttClient);
char line_receive[256];
byte mon_index = 0;
unsigned long next_mqtt_test_connection;


#define MENU    0
#define LINK    1
#define SLIDER  2
#define PLOT    3

#define DEBUG 1

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

PN5180ISO15693 nfc(12, 13, 14);
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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}



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
  WiFi.setSleep(false); //SD

  setLedSequence(ledSeq_connecting);

    
  if(!SD.begin()){
        Serial_ESP_to_USB.println(MSG_HEADER" Card Mount Failed");
        if (!SPIFFS.begin()) {
            Serial_ESP_to_USB.println(MSG_HEADER" SPIFFS not initialized! Stop!");
            //while (1) yield();
        }
        else {
          // Die Abfrage auf die reine URL '/' wird auf '/index.html' im SPIFFS umgelenkt
          server.serveStatic("/", SPIFFS, "/index.html");
          Serial_ESP_to_USB.println(MSG_HEADER" SPIFFS initialized!");
        }
        //return;
  }
  else {
        uint8_t cardType = SD.cardType();
      
        if(cardType == CARD_NONE){
            Serial_ESP_to_USB.println(MSG_HEADER" No SD card attached");
            //return;
        }
      
        Serial_ESP_to_USB.print(MSG_HEADER" SD Card Type: ");
        if(cardType == CARD_MMC){
            Serial_ESP_to_USB.println(MSG_HEADER" MMC");
        } else if(cardType == CARD_SD){
            Serial_ESP_to_USB.println(MSG_HEADER" SDSC");
        } else if(cardType == CARD_SDHC){
            Serial_ESP_to_USB.println(MSG_HEADER" SDHC");
        } else {
            Serial_ESP_to_USB.println(MSG_HEADER" UNKNOWN");
        }
      
        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial_ESP_to_USB.printf("SD Card Size: %lluMB\n", cardSize);
        listDir(SD, "/", 3);
       
      
        Serial_ESP_to_USB.printf(MSG_HEADER" Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
        Serial_ESP_to_USB.printf(MSG_HEADER" Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
        
        // Die Abfrage auf die reine URL '/' wird auf '/index.html' umgelenkt
        server.serveStatic("/", SD, "/index.html");
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
  if (DEBUG) Serial_ESP_to_USB.println("Starting PFODServeur on port 8881");
  PFODserver.begin();
  PFODserver.setNoDelay(true);
ArduinoOTASetup();

  //bber1
//BT seial for Pfod init
  /*
  Serial.println("Start Bluetooth");
  ESP_BT.begin("NEW_PCB1");
  Serial.println("Bluetooth started with name NEW_PCB  Wait 10 seconde......");
  delay(1000);
  
  */
  
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
  client_mqtt.setServer(mqtt_server, mqtt_port);
  client_mqtt.setCallback(receivedCallback);
}

void loop() {

  Check_WIFI();
  server.handleClient();  //Webserver
  handlePFODclient();

  ArduinoOTA.handle();                 // OTA Upload via ArduinoIDE


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
  if ((useMqtt) && (!client_mqtt.connected()) && (millis() > next_mqtt_test_connection))
  {
    next_mqtt_test_connection = millis() + 5000;
    mqttconnect();
  }
  if (useMqtt) client_mqtt.loop();
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
void ArduinoOTASetup()
{
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial_ESP_to_USB.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial_ESP_to_USB.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial_ESP_to_USB.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial_ESP_to_USB.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial_ESP_to_USB.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial_ESP_to_USB.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial_ESP_to_USB.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial_ESP_to_USB.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial_ESP_to_USB.println("End Failed");
    });

  ArduinoOTA.begin();
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
  client_mqtt.publish(outTopic1, val2);
  //state
  char outTopic2[strlen(mower_name) + strlen(mqtt_stateTopic)];
  sprintf(outTopic2, "%s%s", mower_name, mqtt_stateTopic);
  client_mqtt.publish(outTopic2, val3);
  //temp
  char outTopic3[strlen(mower_name) + strlen(mqtt_tempTopic)];
  sprintf(outTopic3, "%s%s", mower_name, mqtt_tempTopic);
  client_mqtt.publish(outTopic3, val4);
  //battery
  char outTopic4[strlen(mower_name) + strlen(mqtt_batteryTopic)];
  sprintf(outTopic4, "%s%s", mower_name, mqtt_batteryTopic);
  client_mqtt.publish(outTopic4, val5);
  //idle
  char outTopic5[strlen(mower_name) + strlen(mqtt_idleTopic)];
  sprintf(outTopic5, "%s%s", mower_name, mqtt_idleTopic);
  client_mqtt.publish(outTopic5, val6);
}
void mqttconnect() {

  if (!client_mqtt.connected()) {
    if (debug_mqtt) Serial.print("MQTT connecting ...");

    if (client_mqtt.connect(mqtt_id, mqtt_user, mqtt_pass)) {
      if (debug_mqtt) Serial.println("connected");
      char outMessage[strlen(mower_name) + strlen(mqtt_subscribeTopic1)];
      sprintf(outMessage, "%s%s", mower_name, mqtt_subscribeTopic1);
      if (debug_mqtt) Serial.print("Subscribe to : ");
      if (debug_mqtt) Serial.println(outMessage);
      client_mqtt.subscribe(outMessage);

    } else {
      if (debug_mqtt) Serial.print("mqtt failed, status code =");
      if (debug_mqtt) Serial.print(client_mqtt.state());
      if (debug_mqtt) Serial.println("try again in 5 seconds");

    }
  }
}
