#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <detail/RequestHandlersImpl.h>
#include <Ticker.h>
#include <SPIFFS.h>
#include <esp_wifi.h>

#include <ArduinoOTA.h>                // OTA Upload via ArduinoIDE


#define LED 2



#define SERIAL_BUFFER_SIZE 256

#define MAX_CONFIG_LEN  100
#define MSG_HEADER "[ESP]"
#define PFODtoESP "[PFOD->ESP] "
#define ESPtoPFOD "[ESP->PFOD] "
#define WebtoESP "[Web->ESP] "
#define ESPtoWeb "[ESP->Web] "
#define ESPtoPCB "[ESP->PCB] "
#define PCBtoESP "[PCB->ESP] "
#define mqtt-Receive "[mqtt-Receive]"
#define mqtt-Send "[mqtt-Send]"
#define VERSION " Version from 22.05.2022 for SPIFFS and SD-Card Use "
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

IPAddress myIP(10,0,0,168);
IPAddress network_gateway(10,0,0,1);
IPAddress network_subnet(255, 255, 255, 0);
IPAddress network_dns(10,0,0,1);
char *wifi_network_ssid     = "ASUS_38_2G";
char *wifi_network_password = "basicsheep714";

bool wifiConnected = false;
int connectCnt = 0;
WebServer server(80); // Webserver-Instanz für Port 80 erstellen
WiFiServer PFODserver(8080); // TCPserver-Instanz für PFOD-App auf Port 8080 erstellen

WiFiClient PFODclient;
bool PFODclientConnected = false;


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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
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

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
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
  Serial_ESP_to_USB.println(MSG_HEADER" ESP32 Serial_ESP_to_USB WIFI Bridge with Webinterface for AzuritBer ");
  Serial_ESP_to_USB.println(MSG_HEADER" Developed by great software engineers, only slightly modified by Prince Ruprecht");
  Serial_ESP_to_USB.println(MSG_HEADER""VERSION);

  connectWIFI();
  WiFi.setSleep(false); //SD

  setLedSequence(ledSeq_connecting);
  
  if(!SD.begin()){
        Serial_ESP_to_USB.println(MSG_HEADER" SD-Card Mount Failed");
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
  
  // Files ausliefern
  server.on("/", HTTP_GET, handleRoot);
  // LED steuern
  server.on("/set", HTTP_POST, handleSet);
  // Werte abrufen
  server.on("/werte", HTTP_POST, handleWerte);

  server.begin(); // Web-Server starten
  Serial_ESP_to_USB.println(MSG_HEADER " ESP32-Web-Server running on Port 80  ");

  flushInput();

  // Start PFODserver
  PFODserver.begin();
  PFODserver.setNoDelay(true);
  Serial_ESP_to_USB.println(MSG_HEADER " PFOD-Server running on Port 8080  ");

  ArduinoOTASetup();
}

void loop() {

  Check_WIFI();
  server.handleClient();  //Webserver
  handlePFODclient();
  handleSerialInput();

  ArduinoOTA.handle();                 // OTA Upload via ArduinoIDE

  if (millis() > timer1s + 1000UL) {

    timer1s = millis();
  }
}

void handleSerialInput() {
  if (PFODclientConnected) {
    //debugln(MSG_HEADER "PFODclientConnected");
    if (Serial_ESP_to_PCB.available()) {
      debugln(MSG_HEADER "PFODclient as data");
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
    Serial_ESP_to_USB.print(msg);
  }
}

void debugln(String msg) {
  if (DEBUG) {    
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
