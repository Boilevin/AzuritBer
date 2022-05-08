long timeout = 0UL;
long timeout2 = 0UL;
#define TIMEOUT_TIME  10000UL

#define bufferSize 1024
uint8_t WIFIbuf[bufferSize];
uint16_t inWiFI = 0;




void handlePFODclient() {
  // Handle Client connection
  if (PFODclientConnected) {
    if (!PFODclient.connected())  {
      // Client is disconnected
      disconnectPFODClient();
      setLedSequence(ledSeq_connected);
      Serial_ESP_to_USB.println(MSG_HEADER " Client Disconnected");
    }
  }

  if (PFODserver.hasClient()) {
    // A new client tries to connect
    if (!PFODclientConnected) {
      // OK accept the client
      PFODclient = PFODserver.available();
      PFODclientConnected = true;
      timeout = millis();
      setLedSequence(ledSeq_clientConnected);
      Serial_ESP_to_USB.println(MSG_HEADER " PFODClient Connected");
    } else {
      // A client is already connected, refuse
      WiFiClient serverClient = PFODserver.available();
      serverClient.stop();
    }
  }

  if (PFODclientConnected) {
    // Send all bytes received form the client to the Serial Port
    timeout = millis();
    if (PFODclient.available()) {
      while (PFODclient.available())
      {
        WIFIbuf[inWiFI] = PFODclient.read(); // read char from client
        if (inWiFI < bufferSize - 1) inWiFI++;
      }
      Serial2.write(WIFIbuf, inWiFI); // now send to UART(2):
      inWiFI = 0;



      
    }
  }
  if (Serial2.available())
  {
    while (Serial2.available())
    {
      WIFIbuf[inWiFI] = Serial2.read(); // read char from UART(2)
      char aChar = WIFIbuf[inWiFI];
      if (inWiFI < bufferSize - 1) inWiFI++;
      
      if (aChar == '\n')
      {
        // End of record detected. Time to parse and check for non pfod sentence
        //here data coming from teensy for mqtt or start sender
        
        if (strncmp(line_receive, "#SENDER", 7) == 0) {
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
    if (PFODclient)
      PFODclient.write(WIFIbuf, inWiFI);
    inWiFI = 0;
  }
  //Timeout for Connection
  // if ( millis() > timeout + TIMEOUT_TIME)disconnectPFODClient();

}

void disconnectPFODClient(void) {
  PFODclient.stop();
  PFODclientConnected = false;
  flushInput();
}
