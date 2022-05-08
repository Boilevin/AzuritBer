// Werte an Browser senden
void handleWerte() {
  debugln("handleWerte");

  int anzahl = server.args();
  String erg = "";
  if (!PFODclientConnected) {
    if (anzahl == 2) {
      //Zwei Parameter werden erwartet: z.b. cmd={s1}&refresh=true
      if (server.hasArg("cmd") && server.hasArg("refresh")) {
        //Parameter vorhanden

        if (!PFODclientConnected) {
          Serial_ESP_to_PCB.println("{" + server.arg(0) + "}"); //Kommando an Ardumower senden
          String buf = Serial_ESP_to_PCB.readStringUntil('}');
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

        debugln(erg);
        server.send( 200, "text/plain", erg );
      } else {
        server.send(500, "text/plain", "falsche Parameter: z.B. cmd=s1&refresh=true wird erwartet!");
      }
    } else {
      server.send(500, "text/plain", "zuwenig Parameter");
    }
  } else {
    erg = "con=TeensyMower connected to PFOD-App...";
    debugln(erg);
    server.send( 200, "text/plain", erg );
  }
}

void handleSet() {
  debugln("handleSet");
  int anzahl = server.args();
  String erg = "";
  for (int i = 0; i < anzahl; i++) {
    if (server.arg(i) == "true") {
      //Kommando kommt von einem button
      Serial_ESP_to_PCB.println("{" + server.argName(i) + "}"); //Kommando an Ardumower senden

    } else {
      //kommt von einem slider
      Serial_ESP_to_PCB.println("{" + server.argName(i) + "`" + server.arg(i) + "}"); //Kommando an Ardumower senden
    }

    if (anzahl == 0) {
      server.send(500, "text/plain", "zuwenig Parameter");
    } else server.send( 200, "text/plain", "OK" );
  }
}

// File an Browser senden
bool handleFileRead(String path) {
  debugln("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";
  String contentType = StaticRequestHandler::getContentType(path);
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

// Rootverzeichnis
void handleRoot() {
  debugln("handleRoot");
  if (!handleFileRead(server.uri()))  server.send(404, "text/plain", "FileNotFound");
}

String getElementValue(String cmd) {
  for (int i = 0; i < nElemente; i++) {
    if (elemente[i].cmd == cmd) {
      return elemente[i].value;
    }
    return "na";
  }

}
