
#define BLUETOOTH
//Select one of the 2 possibility AP (access point) or STA (station)
#define MODE_AP // phone connects directly to ESP32 inside the mower 
//#define MODE_STA // ESP32 and phone connects to wifi routeur
#define PROTOCOL_TCP
#define bufferSize 1024

bool debug = true;

#define VERSION "1.00"

#ifdef MODE_STA
// For standard mode:
//you need to set a fix IP and gateway according to your routeur value
// ssid and password according your routeur
const char *ssid = "Your ssid";  // You will connect your phone to this Access Point
const char *pw = "Your password"; // and this is the password
IPAddress ip(10, 0, 0, 122); //
IPAddress gateway(10, 0, 0, 1); //
IPAddress netmask(255, 255, 255, 0);
#endif


#ifdef MODE_AP
// For AP mode:
// Don't forget to connect your WIFI phone to the AP :

const char *ssid = "Ardumower";  // You will connect your phone to this Access Point
const char *pw = ""; // no password
IPAddress ip(192, 168, 4, 1); //
IPAddress gateway(192, 168, 4, 0); //
IPAddress netmask(255, 255, 255, 0);
#endif
