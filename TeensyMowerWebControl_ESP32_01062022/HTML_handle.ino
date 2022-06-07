// Werte an Browser senden
void handleWerte() {
  int anzahl = server.args();
  String erg = "";
  if (!PFODclientConnected) {
    if (anzahl == 2) {
      //Zwei Parameter werden erwartet: z.b. cmd={s1}&refresh=true
      if (server.hasArg("cmd") && server.hasArg("refresh")) {
        //Parameter vorhanden

        if (!PFODclientConnected) {
          Serial_ESP_to_PCB.println("{" + server.arg(0) + "}"); //Kommando an Ardumower senden
          debug(ESPtoPCB); debugln("{" + server.arg(0) + "}");
          String buf = Serial_ESP_to_PCB.readStringUntil('}');
          debug(PCBtoESP); debugln("reading Serial Connection PCB->ESP");
          debug(PCBtoESP); debugln(buf);
          //buf = msgTest3;
          if (buf != "") {
            //Daten empfangen
            parse_PFOD(buf);
            erg = "con=TeensyMower connection OK...&";
            for (int i = 0; i < nElemente; i++) {
              if (elemente[i].type == SLIDER) {
                if (server.arg(1) == "false" || DEBUG == 1) {
                  //Paramtersatz vollständig übertragen
                  erg += elemente[i].cmd + "_min=" + elemente[i].sl_min + "&";
                  erg += elemente[i].cmd + "_max=" + elemente[i].sl_max + "&";
                  erg += elemente[i].cmd + "_res=" + elemente[i].sl_res + "&";
                }
                erg += elemente[i].cmd + "_val=" + elemente[i].value;
                if (i < nElemente - 1) erg += "&"; //es gibt weitere elemente
              } else if (elemente[i].type == LINK) {
                erg += elemente[i].cmd + "=" + elemente[i].value;
                if (i < nElemente - 1) erg += "&"; //es gibt weitere elemente
              }
            }//for elemente
          } else {
            erg = "con=TeensyMower not connected   ";
          }
        }        
        server.send( 200, "text/plain", erg );
        debug(ESPtoWeb); debugln(erg);
      } else {
        server.send(500, "text/plain", "falsche Parameter: z.B. cmd=s1&refresh=true wird erwartet!");
        debug(ESPtoWeb); debugln(erg);
      }
    } else {
      server.send(500, "text/plain", "zuwenig Parameter");
      debug(ESPtoWeb); debugln(erg);
    }
  } else {
    erg = "con=TeensyMower connected to PFOD-App...";
    server.send( 200, "text/plain", erg );
    debug(ESPtoWeb); debugln(erg);
  }
}

void handleSet() {
  int anzahl = server.args();
  String erg = "";
  for (int i = 0; i < anzahl; i++) {
    if (server.arg(i) == "true") {
      //Kommando kommt von einem button
      Serial_ESP_to_PCB.println("{" + server.argName(i) + "}"); //Kommando an Ardumower senden
      debug(ESPtoPCB);debugln("{" + server.argName(i) + "}");

    } else {
      //kommt von einem slider
      Serial_ESP_to_PCB.println("{" + server.argName(i) + "`" + server.arg(i) + "}"); //Kommando an Ardumower senden
      debug(ESPtoPCB);debugln("{" + server.argName(i) + "}");
    }

    if (anzahl == 0) {
      server.send(500, "text/plain", "zuwenig Parameter");
      debug(ESPtoWeb); debugln("zuwenig Parameter");
    } else {
      server.send( 200, "text/plain", "OK" );
      debug(ESPtoWeb); debugln("OK");
    }
  }
}

// File an Browser senden
bool handleFileRead(String path) {
  debugln(ESPtoWeb "handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";
  String contentType = StaticRequestHandler::getContentType(path);
  if (SD.exists(path)) {
    File file = SD.open(path, "r");  
    size_t sent = server.streamFile(file, contentType);
    file.close();
    debugln(MSG_HEADER" html file loaded from sd card");
    return true;
  }    
  else if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    debugln(MSG_HEADER" html file loaded from SPIFFS");
    return true;
  }
  else return false;
}

// Rootverzeichnis
void handleRoot() {
  debugln(MSG_HEADER" handleRoot");
  if (!handleFileRead(server.uri()))  {
    server.send(404, "text/plain", "FileNotFound");
    debug(ESPtoWeb); debugln("FileNotFound");
  }
}

String getElementValue(String cmd) {
  for (int i = 0; i < nElemente; i++) {
    if (elemente[i].cmd == cmd) {
      return elemente[i].value;
    }
    return "na";
  }

}

void handleNotFound() {
  // Output a "404 not found" page. It includes the parameters which comes handy for test purposes.
  Serial.println(F("TeensyMowerWebControl handleNotFound()"));
  String message;
  message += F("404 - File Not Found\n"
               "URI: ");
  message += server.uri();
  message += F("\nMethod: ");
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += F("\nArguments: ");
  message += server.args();
  message += F("\n");
  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  debug(ESPtoWeb); debugln(message);
}


void handle204()
{
  server.send(204);                // this page doesn't send back content
}
