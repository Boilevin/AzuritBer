#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <detail/RequestHandlersImpl.h>
#include <Ticker.h>
#include <SPIFFS.h>
#include <esp_wifi.h>


#define dataSerial Serial2
#define debugSerial Serial
#define BAUDRATE 19200  // use 115200 for teensy

IPAddress myIP(10, 0, 0, 168);
IPAddress network_gateway(10, 0, 0, 1);
IPAddress network_subnet(255, 255, 255, 0);
IPAddress network_dns(10, 0, 0, 1);
char *wifi_network_ssid     = "WAVLINK-N";
char *wifi_network_password = "basicsheep713";


#define LED 2


#define SERIAL_BUFFER_SIZE 256

#define MAX_CONFIG_LEN  100
#define MSG_HEADER "[WSB]"
#define VERSION "vom 15.03.2022"
#define CONFIG_MSG_START "config:"


#define SET_IP_SETTING  1  //true=IP-Settings aus programm, false=IP-Settings von Mower


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
  dataSerial.begin(BAUDRATE);
  dataSerial.setTimeout(500);

  debugSerial.begin(115200);
  debugSerial.setTimeout(500);


  // Configure LED
  pinMode(LED, OUTPUT);
  setLedSequence(ledSeq_startup);
  ledTicker.attach(0.1, onLedTicker);

  // Welcome message
  delay(500);
  debugSerial.println("\n\n");
  debugSerial.println(MSG_HEADER " ESP32 Serial WIFI Bridge with Webinterface for AzuritBer ! (not azurit) " VERSION);

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
}

void loop() {

  Check_WIFI();
  server.handleClient();  //Webserver
  handlePFODclient();
  handleSerialInput();

  if (millis() > timer1s + 1000UL) {

    timer1s = millis();
  }
}

void handleSerialInput() {
  if (PFODclientConnected) {
    if (dataSerial.available()) {
      size_t len = min(dataSerial.available(), 255);
      char sbuf[len];
      String erg = "";
      dataSerial.readBytes(sbuf, len);
      PFODclient.write(sbuf, len);

    }
  }
}

void debug(String msg) {
  if (DEBUG) {
    dataSerial.print(MSG_HEADER);
    dataSerial.print(" ");
    dataSerial.print(msg);
  }
}

void debugln(String msg) {
  if (DEBUG) {
    dataSerial.print(MSG_HEADER);
    dataSerial.print(" ");
    dataSerial.println(msg);
  }
}

void flushInput(void) {
  while (dataSerial.available())
    dataSerial.read();
}
