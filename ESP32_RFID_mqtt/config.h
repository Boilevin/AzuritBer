// SETTING
//If you don't have wifi in all the garden:
//uncomment #define MODE_AP and comment #define MODE_STA
//pfod setting :  IP 192.168.1.4 port 8881 -> mower is the Access point 
//pfod and perimeter sender connect to the mower , adjust sender ssid and pass to the mower AP
//Mqtt can't work you need to deactivate it

//If you have wifi in all the garden:
//comment #define MODE_AP and uncomment #define MODE_STA
//set your ssid and password acoording to your WIFI router
//pfod setting : IP 10.0.0.122 (or change to correct IPgroup of your router) port 8881 -> mower and sender are connected to your home wifi router
//perimeter sender is connected to your WIFI router so set the same IP adress location (eg 10.0.0.123 /100.0.0.124 etc...) inside the sender software

//Bluetooth:
//limitation no automatic start and stop sender on multiple mowing area no mqtt
//Sender need the autostart option activated
//Mqtt can't work you need to deactivate it

//*********WIFI***************//
//Uncomment only one of the 2 possibility : MODE_AP (access point) or MODE_STA (station)
//#define MODE_AP // phone connects directly to ESP32 inside the mower IP: 192.168.4.1 port 8881
#define MODE_STA // ESP32 and phone connects to wifi routeur
const char *ssid = "your ssid";  // Your Routeur : You need to connect your phone to the same Access Point to use PfodApp
const char *pw = "your_pass"; // and this is the password

IPAddress ip(10, 0, 0, 123); //you need to set a fix IP according to your routeur value
IPAddress gateway(10, 0, 0, 1); //
IPAddress netmask(255, 255, 255, 0);

//*********RFID**************//
#define rfid_board_IsPluged true //Set to true or false according your hardware

//*********MQTT*************//
#define useMqtt true //Set to true or false
const char* mower_name = "Rl1000";//root help if you have many mower connected to the same broker
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


bool debug = true;
#define BLUETOOTH //comment this line if you don't want to use BLUETOOTH


#define VERSION "1.20"
#define my_bufferSize 1024
#ifdef MODE_AP
const char *ssid = "TeensyMower";  // You will connect your phone to this Access Point
const char *pw = ""; // no password
//for pfod use this IP and port 8881
IPAddress ip(192, 168, 4, 1); //
IPAddress gateway(192, 168, 4, 0); //
IPAddress netmask(255, 255, 255, 0);
#endif
