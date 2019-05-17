#include <WiFi.h>


const char* ssid     = "ArduWifi";   // put here your phone acces point ssid
const char* password = "Ardumower1234";  // put here the password


byte sigCodeInUse = 2;
int8_t sigcode_norm[128];
int sigcode_size;



hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



#define pinIN1       12  // M1_IN1         ( connect this pin to L298N-IN1)
#define pinIN2       13  // M1_IN2         ( connect this pin to L298N-IN2)

#define pinEnable    23  // EN             (connect to motor driver enable)    

#define pinFeedback      38  //           (connect to INA169 OUT)
#define USE_PERI_CURRENT      0     // use pinFeedback for perimeter current measurements? (set to '0' if not connected!)
#define PERI_CURRENT_MIN    0.1    // minimum Ampere for perimeter-is-closed detection 
#define WORKING_TIMEOUT_MINS 300  // timeout for perimeter switch-off if robot not in station (minutes)
#define LED_BUILTIN 2

// code version
#define VER "ESP32 1.0"

int step = 0;
boolean enableSender = true;
boolean WiffiRequestOn = true;
int sigDuration = 104;


int timeSeconds = 0;
unsigned long nextTimeControl = 0;
unsigned long nextTimeInfo = 0;
unsigned long nextTimeSec = 0;
int workTimeMins = 0;
int periCurrentAnalogIn;


// must be multiple of 2 !
// http://grauonline.de/alexwww/ardumower/filter/filter.html
// "pseudonoise4_pw" signal (sender)
int8_t sigcode0[] = { 1, -1 };  //simple square test code
int8_t sigcode1[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };  //ardumower signal code
int8_t sigcode2[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1 };
int8_t sigcode3[] = { 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1,
                      -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1
                    }; // 128 Zahlen from Roland
int8_t sigcode4[] = { 1, 1, 1, -1, -1, -1};  //extend square test code

WiFiServer server(80);


void IRAM_ATTR onTimer()
{ // management of the signal
  portENTER_CRITICAL_ISR(&timerMux);
  if (enableSender) {

    if (sigcode_norm[step] == 1) {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, HIGH);

    } else if (sigcode_norm[step] == -1) {
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, LOW);

    } else {
      Serial.println("errreur");
      //digitalWrite(pinEnable, LOW);
    }
    step ++;
    if (step == sigcode_size) {
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
  step = 0;
  enableSender = false;
  Serial.print("Change to Area : ");
  Serial.println(areaInMowing);
  for (int uu = 0 ; uu <= 128; uu++) { //clear the area
    sigcode_norm[uu] = 0;
  }
  sigcode_size = 0;
  switch (areaInMowing) {
    case 0:
      sigcode_size = sizeof sigcode0;
      for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode0[uu];
      }
      break;
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
    case 4:
      sigcode_size = sizeof sigcode4;
      for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode4[uu];
      }
      break;


  }

  Serial.print("New sigcode in use  : ");
  Serial.println(sigCodeInUse);

  for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
    Serial.print(sigcode_norm[uu]);
    Serial.print(",");
  }
  Serial.println();
  Serial.print("New sigcode size  : ");
  Serial.println(sigcode_size);
  enableSender = true;
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
  pinMode(LED_BUILTIN, OUTPUT);

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

  changeArea(sigCodeInUse);


  //------------------------  WIFI parts  ----------------------------------------


  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 MacAdress use it to reserve IP 10.42.1.254 on DHCP Raspberry Pi because it's reserve IP Fix for this sender -------------------> ");
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
  delay(100);

}
void connection()
{


  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //WiFi.setAutoReconnect(true);
  /*
    IPAddress local_IP(10, 42, 0, 254);
    IPAddress gateway(10, 42, 0, 1);
    IPAddress subnet(255, 255, 0, 0);
    if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure with the fix IP ");
    }
  */

  WiFi.begin(ssid, password);

  for (int i = 0; i < 60; i++)
  {
    if ( WiFi.status() != WL_CONNECTED )
    {
      Serial.println("Try connecting...");
      delay (250);

    }
  }


  if ( WiFi.status() == WL_CONNECTED )
  {
    Serial.println("Mower connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    //String ipAddress = IPAddress2String(WiFi.localIP());


    server.begin();
  }



}
static void ScanNetwork()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.println("Disconnect All ");

  int n = WiFi.scanNetworks();

  if (n == -1)
  {
    Serial.println("Scan running ???");
  }
  if (n == -2)
  {
    Serial.println("Scan Fail.");
  }
  if (n == 0)
  {
    Serial.println("No networks.");
  }
  if (n > 0)
  {
    Serial.print("find ");
    Serial.print(n);
    Serial.println(" Acces point");
    for (int i = 0; i < n; ++i) {
      // Print SSID for each network found
      char currentSSID[64];
      WiFi.SSID(i).toCharArray(currentSSID, 64);
      Serial.print("find Wifi : ");
      Serial.println(currentSSID);
      //delay (1500);
      if (String(currentSSID) == ssid)
      {
        connection();
        i = 200; //to avoid loop again when connected
      }
      else
      {
        Serial.println("But it's not the Mower");
      }
    }
  }


}


void loop()
{


  if (millis() >= nextTimeControl) {
    nextTimeControl = millis() + 2000;  //after debug can set this to 10 secondes
    //Serial.print("worktime= ");
    //Serial.println(workTimeMins);

    if (USE_PERI_CURRENT) {
      periCurrentAnalogIn = analogRead(pinFeedback);
      Serial.print("Pericurr= ");
      Serial.println(periCurrentAnalogIn);
    }



    if ( (WiFi.status() != WL_CONNECTED)) ScanNetwork(); // warning it's blocking fonction for 2 secondes
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

  }

  if (millis() >= nextTimeInfo) {
    nextTimeInfo = millis() + 500;
    float v = 0;

    digitalWrite(LED_BUILTIN, enableSender);   // turn the Blue LED on if sender ON


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
      WiffiRequestOn = 0;
      enableSender = false;
      workTimeMins = 0;
      digitalWrite(pinEnable, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);

      String sResponse;
      sResponse = "SENDER IS OFF";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }
    if (req.indexOf("/area2/1") != -1) {
      WiffiRequestOn = 1;
      workTimeMins = 0;
      enableSender = true;
      digitalWrite(pinEnable, HIGH);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      // Prepare the response
      String sResponse;
      sResponse = "SENDER IS ON";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }

    if (req.indexOf("/sigCode/0") != -1) {
      sigCodeInUse = 0;
      changeArea(sigCodeInUse);
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 0";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }
    if (req.indexOf("/sigCode/1") != -1) {
      sigCodeInUse = 1;
      changeArea(sigCodeInUse);
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 1";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }
    if (req.indexOf("/sigCode/2") != -1) {
      sigCodeInUse = 2;
      changeArea(sigCodeInUse);
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 2";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }

    if (req.indexOf("/sigCode/3") != -1) {
      sigCodeInUse = 3;
      changeArea(sigCodeInUse);
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 3";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }

    if (req.indexOf("/sigCode/4") != -1) {
      sigCodeInUse = 4;
      changeArea(sigCodeInUse);
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 4";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }



    if (req.indexOf("/sigDuration/104") != -1) {
      sigDuration = 104;
      timerAlarmWrite(timer, 104, true);
      // Prepare the response
      String sResponse;
      sResponse = "NOW 104 microsecond signal duration";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }

    if (req.indexOf("/sigDuration/50") != -1) {
      sigDuration = 50;
      timerAlarmWrite(timer, 50, true);
      // Prepare the response
      String sResponse;
      sResponse = "NOW 50 microsecond signal duration";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();

    }












    if (req.indexOf("/?") != -1) {
      String sResponse, sHeader;
      sResponse = "WORKING DURATION= ";
      sResponse += workTimeMins ;
      sResponse += " PERI CURRENT Milli Amps= ";
      sResponse += periCurrentAnalogIn ;
      sResponse += " sigDuration= ";
      sResponse += sigDuration ;
      sResponse += " sigCodeInUse= ";
      sResponse += sigCodeInUse ;


      sigDuration = 104;
      // Send the response to the client
      client.print(sResponse);
      client.flush();
    }


  }













}
