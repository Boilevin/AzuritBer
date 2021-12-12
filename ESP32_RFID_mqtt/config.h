// SETTING
//If you don't have wifi in all the garden:
//set MODE_AP to true and  MODE_STA to false
//pfod setting :  IP 192.168.4.1 port 8881 -> mower is the Access point
//pfod and perimeter sender connect to the mower , adjust sender ssid and pass to the mower AP
//Mqtt can't work you need to deactivate it

//If you have wifi in all the garden:
//set MODE_AP to false and  MODE_STA to true
//set your ssid and password acoording to your WIFI router
//pfod setting : IP 10.0.0.122 (or change to correct IPgroup of your router) port 8881 -> mower and sender are connected to your home wifi router
//perimeter sender is connected to your WIFI router so set the same IP adress location (eg 10.0.0.123 /100.0.0.124 etc...) inside the sender software


//Bluetooth:
//limitation no automatic start and stop sender on multiple mowing area no mqtt
//Sender need the autostart option activated
//Mqtt can't work you need to deactivate it

//*********WIFI***************//
//set to true and false one of the 2 possibility : MODE_AP (access point) or MODE_STA (station)

boolean MODE_STA = true; // ESP32 and phone connects to wifi routeur and auto switch to mode_ap if fail
boolean MODE_AP = false; //  phone connects directly to ESP32 inside the mower IP: 192.168.4.1 port 8881

const char *ssid = "your ssid";  // Your Routeur : You need to connect your phone to the same Access Point to use PfodApp
const char *pw = "your pass"; // and this is the password

IPAddress ip(10, 0, 0, 123); //you need to set a fix IP according to your routeur value
IPAddress gateway(10, 0, 0, 1); //
IPAddress netmask(255, 255, 255, 0);

//*********RFID**************//
#define rfid_board_IsPluged false //Set to true or false according your hardware

//*********MQTT*************//
boolean useMqtt = false ; //Set to true only if you have a mosquito broker running in your house or false
const char* mower_name = "Teensy2";//root help if you have many mower connected to the same broker
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


#define VERSION "1.30"
#define my_bufferSize 1024

//acces point credential
const char *ssid_ap = "ROBOTMOWER";  // You will connect your phone to this Access Point
const char *pw_ap = ""; // no password
//for pfod use this IP and port 8881
IPAddress ip_ap(192, 168, 4, 1); //
IPAddress gateway_ap(192, 168, 4, 0); //
IPAddress netmask_ap(255, 255, 255, 0);
