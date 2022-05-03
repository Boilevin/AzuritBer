long timeout = 0UL;
long timeout2 = 0UL;
#define TIMEOUT_TIME  10000UL


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
    if (PFODclient.available()) {
      timeout = millis();

      String request = PFODclient.readStringUntil('}');
        Serial_ESP_to_USB.println(request);
        String response = Serial_ESP_to_PCB.readStringUntil('}');
        PFODclient.println(response);

      while (PFODclient.available()) {
        size_t len = min(PFODclient.available(), 255);
        uint8_t sbuf[len];
        PFODclient.read(sbuf, len);
        Serial_ESP_to_PCB.write(sbuf, len);
      }
    }
  }

  //Timeout for Connection
  if ( millis() > timeout + TIMEOUT_TIME)disconnectPFODClient();

}

void disconnectPFODClient(void) {
  PFODclient.stop();
  PFODclientConnected = false;
  flushInput();
}
