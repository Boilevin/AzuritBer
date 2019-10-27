
#include <U8x8lib.h>
#include <WiFi.h>


const char* ssid     = "Arduwifi";   // put here your phone acces point ssid
const char* password = "Ardumower1234";  // put here the password

//const char* ssid     = "Androidbb";   // put here your phone acces point ssid
//const char* password = "rvhm0948";  // put here the password


int8_t sigcode_norm[128];

int16_t sigcode_size;



// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);


hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;




// ---- choose only one perimeter signal code ----
//#define SIGCODE_0  // test scope signal
#define SIGCODE_1  // Ardumower default perimeter signal
//#define SIGCODE_2  // BB alternative perimeter signal
//#define SIGCODE_3  // Ardumower alternative perimeter signal
//#define SIGCODE_4  // Raindancer perimeter signal

#define pinIN1       12  // M1_IN1         ( connect this pin to L298N-IN1)
#define pinIN2       13  // M1_IN2         ( connect this pin to L298N-IN2)

#define pinEnable    23  // EN             (connect to motor driver enable)    

#define pinFeedback      38  //           (connect to INA169 OUT)
#define USE_PERI_CURRENT      1     // use pinFeedback for perimeter current measurements? (set to '0' if not connected!)
#define PERI_CURRENT_MIN    0.1    // minimum Ampere for perimeter-is-closed detection 
#define WORKING_TIMEOUT_MINS 300  // timeout for perimeter switch-off if robot not in station (minutes)


// code version
#define VER "ESP32 1.0"

volatile int step = 0;
boolean enableSender = false; //OFF on start to autorise the reset
//boolean WiffiRequestOn = true;



int timeSeconds = 0;
unsigned long nextTimeControl = 0;
unsigned long nextTimeInfo = 0;
unsigned long nextTimeSec = 0;
int workTimeMins = 0;
int periCurrentAnalogIn;


// must be multiple of 2 !
// http://grauonline.de/alexwww/ardumower/filter/filter.html
// "pseudonoise4_pw" signal (sender)
#if defined (SIGCODE_0)
int8_t sigcode[] = { 1, -1 };
#elif defined (SIGCODE_1)
int8_t sigcode[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
#elif defined (SIGCODE_2)
int8_t sigcode[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1 };
#elif defined (SIGCODE_3)
int8_t sigcode[] = { 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1 };
#elif defined (SIGCODE_4)
int8_t sigcode[] = { 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1,
                     -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1
                   }; // 128 Zahlen from Roland


#endif



int8_t sigcode0[] = { 1, -1 };
int8_t sigcode1[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int8_t sigcode2[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1 };
int8_t sigcode3[] = { 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1 };
int8_t sigcode4[] = { 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1,
                      -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1
                    }; // 128 Zahlen from Roland

WiFiServer server(80);


void IRAM_ATTR onTimer()
{ // management of the signal
  portENTER_CRITICAL_ISR(&timerMux);
  if (enableSender) {
    if (sigcode[step] == 1) {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, HIGH);

    } else if (sigcode[step] == -1) {
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, LOW);

    } else {
      digitalWrite(pinEnable, LOW);  //if there is a 0 in the sigcode
    }
    step ++;
    if (step == sizeof sigcode) {
      step = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);

}

String IPAddress2String(IPAddress address)
{
  return String(address[0]) + "." +
         String(address[1]) + "." +
         String(address[2]) + "." +
         String(address[3]);
}



void changeArea(byte areaInMowing) {  // not finish to dev
  Serial.print("Change to Area : ");
  Serial.println(areaInMowing);
  for (int uu = 0 ; uu <= 128; uu++) { //clear the area
    sigcode_norm[uu] = 0;

  }
  switch (areaInMowing) {

    case 1:
      sigcode_size = sizeof sigcode1;
      for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode1[uu];

      }

      break;
    case 2:
      sigcode_size = sizeof sigcode2;
      for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode2[uu];

      }

      break;
    case 3:
      sigcode_size = sizeof sigcode3;
      for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode3[uu];

      }

      break;

  }

  Serial.print("New sigcode in use  : ");

  for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
    Serial.print(sigcode_norm[uu]);
    Serial.print(",");
  }
  Serial.println();

}


void setup()
{

  //------------------------  Signal parts  ----------------------------------------
  Serial.begin(115200);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 104, true);
  timerAlarmEnable(timer);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinEnable, OUTPUT);

  pinMode(pinFeedback, INPUT);

  Serial.println("START");
  Serial.print("Ardumower Sender ");
  Serial.println(VER);
#if defined (SIGCODE_1)
  Serial.println("SIGCODE_1");
#elif defined (SIGCODE_2)
  Serial.println("SIGCODE_2");
#elif defined (SIGCODE_3)
  Serial.println("SIGCODE_3");
#elif defined (SIGCODE_4)
  Serial.println("SIGCODE_4");
#endif


  Serial.print("USE_PERI_CURRENT=");
  Serial.println(USE_PERI_CURRENT);

  if (enableSender) {
    digitalWrite(pinEnable, HIGH);
  }


  //------------------------  WIFI parts  ----------------------------------------


  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  //WIFI_ALL_CHANNEL_SCAN
  //esp_wifi_set_config()


  WiFi.disconnect();
  delay(100);

  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);


}
void connection()
{


  u8x8.clearDisplay();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  for (int i = 0; i < 60; i++)
  {
    if ( WiFi.status() != WL_CONNECTED )
    {
      u8x8.drawString(0, 0, "Try connecting");
      delay (250);

    }
  }


  if ( WiFi.status() == WL_CONNECTED )
  {
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    String ipAddress = IPAddress2String(WiFi.localIP());
    u8x8.clearDisplay();
    u8x8.drawString(0, 0, "Mower Connected");
    u8x8.setCursor(0, 1);
    u8x8.print(ipAddress);

    server.begin();
  }



}
static void ScanNetwork()
{
  u8x8.clearDisplay();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  u8x8.drawString(0, 0, "Hotspot Lost");
  if (enableSender) {
      
      u8x8.drawString(0, 3, "Sender ON ");

    }
    else
    {
      u8x8.drawString(0, 3, "Sender OFF");

    }
    u8x8.drawString(0, 4, "worktime= ");
    u8x8.setCursor(10, 4);
    u8x8.print("     ");
    u8x8.setCursor(10, 4);
    u8x8.print(workTimeMins);
  
  delay(5000);  // wait until all is disconnect

  int n = WiFi.scanNetworks();

  if (n == -1)
  {
    u8x8.drawString(0, 0, "Scan running ???");
    u8x8.drawString(0, 1, "Need Reset ???");
    u8x8.drawString(0, 2, "If sender is OFF");

    
    delay(5000);
    if (!enableSender) ESP.restart(); // do not reset if sender is ON
  }
  if (n == -2)
  {
    u8x8.drawString(0, 0, "Scan Fail.");
    u8x8.drawString(0, 1, "Need Reset ???");
    u8x8.drawString(0, 2, "If sender is Off");

    delay(5000);
    if (!enableSender) ESP.restart();
  }
  if (n == 0)
  {
    u8x8.drawString(0, 0, "No networks.");
  }
  if (n > 0)
  {
    Serial.print("find ");
    Serial.println(n);
    u8x8.drawString(0, 0, "Find ");
    for (int i = 0; i < n; ++i) {
      // Print SSID for each network found
      char currentSSID[64];
      WiFi.SSID(i).toCharArray(currentSSID, 64);
      Serial.print("find Wifi : ");
      Serial.println(currentSSID);
      u8x8.drawString(5, 0, currentSSID);
      delay (1500);
      if (String(currentSSID) == ssid) {
        connection();
        i = 200; //to avoid loop again when connected
      }
    }
  }


}


void loop()
{


  if (millis() >= nextTimeControl) {
    nextTimeControl = millis() + 2000;  //after debug can set this to 10 secondes
    u8x8.drawString(0, 4, "worktime= ");
    u8x8.setCursor(10, 4);
    u8x8.print("     ");
    u8x8.setCursor(10, 4);
    u8x8.print(workTimeMins);

    if (USE_PERI_CURRENT) {
      periCurrentAnalogIn = analogRead(pinFeedback);
      u8x8.drawString(0, 5, "Pericurr= ");
      u8x8.setCursor(10, 5);
      u8x8.print("     ");
      u8x8.setCursor(10, 5);
      u8x8.print(periCurrentAnalogIn);
    }


    // if ((!enableSender) && (WiFi.status() != WL_CONNECTED)) ScanNetwork();
    if ( (WiFi.status() != WL_CONNECTED)) ScanNetwork();
    if  ( workTimeMins >= WORKING_TIMEOUT_MINS ) {
      // switch off perimeter
      enableSender = false;
      workTimeMins = 0;
      digitalWrite(pinEnable, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      Serial.println("********************************   Timeout , so stop Sender  **********************************");
    }

  }
  if (millis() >= nextTimeSec) {
    nextTimeSec = millis() + 1000;
    timeSeconds++;
    if ((enableSender) && (timeSeconds >= 60)) {
      if (workTimeMins < 1440) workTimeMins++;
      timeSeconds = 0;
    }
    if (enableSender) {
      u8x8.drawString(0, 2, "Sender ON ");

    }
    else
    {
      u8x8.drawString(0, 2, "Sender OFF");

    }
  }

  if (millis() >= nextTimeInfo) {
    nextTimeInfo = millis() + 500;
    float v = 0;


  }

  // Check if a client has connected
  WiFiClient client = server.available();
  if (client) {
    // Read the first line of the request
    String req = client.readStringUntil('\r');
    Serial.print("Client say  ");
    Serial.println(req);
    Serial.println("------------------------- ");


    //client.flush();

    // Match the request

    if (req.indexOf("/area2/0") != -1) {
      // WiffiRequestOn = false;
      enableSender = false;
      workTimeMins = 0;
      digitalWrite(pinEnable, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);

      String sResponse;
      sResponse = "SENDER IS OFF";
      // Send the response to the client
      client.print(sResponse);
      client.flush();

    }
    if (req.indexOf("/area2/1") != -1) {
      //WiffiRequestOn = 1;
      workTimeMins = 0;
      enableSender = true;
      digitalWrite(pinEnable, HIGH);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      // Prepare the response
      String sResponse;
      sResponse = "SENDER IS ON";
      // Send the response to the client
      client.print(sResponse);
      client.flush();

    }
    if (req.indexOf("/?") != -1) {
      String sResponse, sHeader;
      sResponse = "WORKING DURATION= ";
      sResponse += workTimeMins ;
      sResponse += "    PERI CURRENT Milli Amps= ";
      sResponse += periCurrentAnalogIn ;

      sHeader  = "HTTP/1.1 200 OK\r\n";
      sHeader += "Content-Length: ";
      sHeader += sResponse.length();
      sHeader += "\r\n";
      sHeader += "Content-Type: text/html\r\n";
      sHeader += "Connection: close\r\n";
      sHeader += "\r\n";

      // Send the response to the client
      client.print(sHeader);
      client.print(sResponse);
      client.println();
      client.flush();
    }


  }













}
