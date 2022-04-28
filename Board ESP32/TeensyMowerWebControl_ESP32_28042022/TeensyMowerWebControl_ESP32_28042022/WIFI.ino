void connectWIFI() {
  int warten_auf_wifi=0;
  // Get configuration message
  if (SET_IP_SETTING == 0) {
    Serial_ESP_to_USB.println(MSG_HEADER " Wait for settings from serial...");
    setLedSequence(ledSeq_waitForConfig);
    waitForParams();
    printParams();
    if (strlen(params[PARAMID_LOCALIP].valueStr) > 0) {
      str2IpAddr(params[PARAMID_LOCALIP].valueStr, &myIP);
      str2IpAddr(params[PARAMID_GATEWAY].valueStr, &network_gateway);
      str2IpAddr(params[PARAMID_SUBNET].valueStr, &network_subnet);
    }
    WiFi.mode(WIFI_STA);
    WiFi.config(myIP, network_gateway, network_subnet);    
    WiFi.begin(params[PARAMID_SSID].valueStr, params[PARAMID_PASSWD].valueStr);
    WiFi.setHostname("TeensyMower");
    Serial_ESP_to_USB.println();
    esp_err_t esp_error = esp_wifi_set_ps(WIFI_PS_NONE);Serial_ESP_to_USB.print(MSG_HEADER " esp_wifi_set_ps = ");Serial_ESP_to_USB.println(esp_error);      
    delay(250);    
    while (WiFi.status() != WL_CONNECTED) {    
      delay(500);
      digitalWrite(LED, LOW);
      delay(20);
      digitalWrite(LED, HIGH);
      delay(20); 
      Serial_ESP_to_USB.print(".");
      warten_auf_wifi += 1;
      if (warten_auf_wifi == 20) {
        warten_auf_wifi = 0;
        Serial_ESP_to_USB.println();
        Serial_ESP_to_USB.println("Wifi Reconnect");
        WiFi.reconnect();
        delay(1000);
        WiFi.begin(params[PARAMID_SSID].valueStr, params[PARAMID_PASSWD].valueStr);
        WiFi.setHostname("TeensyMower");
        Serial_ESP_to_USB.println();
        esp_err_t esp_error = esp_wifi_set_ps(WIFI_PS_NONE);Serial_ESP_to_USB.print(MSG_HEADER " esp_wifi_set_ps = ");Serial_ESP_to_USB.println(esp_error);      
      }        
    }
  } else {
    Serial_ESP_to_USB.println(MSG_HEADER " Connecting with programmed settings");
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("TeensyMower");
    WiFi.config(myIP, network_gateway, network_subnet);
    WiFi.begin(wifi_network_ssid, wifi_network_password);
    delay(250);    
    while (WiFi.status() != WL_CONNECTED) {    
      delay(500);
      digitalWrite(LED, LOW);
      delay(20);
      digitalWrite(LED, HIGH);
      delay(20); 
      Serial_ESP_to_USB.print(".");
      warten_auf_wifi += 1;
      if (warten_auf_wifi == 20) {
        warten_auf_wifi = 0;
        Serial_ESP_to_USB.println();
        Serial_ESP_to_USB.println(MSG_HEADER "Wifi Reconnect");
        WiFi.reconnect();
        delay(1000);
        WiFi.begin(wifi_network_ssid, wifi_network_password);
        WiFi.setHostname("TeensyMower");
        Serial_ESP_to_USB.println();
        esp_err_t esp_error = esp_wifi_set_ps(WIFI_PS_NONE);Serial_ESP_to_USB.print(MSG_HEADER " esp_wifi_set_ps = ");Serial_ESP_to_USB.println(esp_error);      
      }        
    }
  }
}

void Check_WIFI() {
  // Handle AccessPoint connection
  if (WiFi.status() == WL_CONNECTED) {
    // Connected to AP
    if (!wifiConnected) {
      // Transition Disconnected => Connected
      Serial_ESP_to_USB.println();
      esp_err_t esp_error = esp_wifi_set_ps(WIFI_PS_NONE);Serial_ESP_to_USB.print(MSG_HEADER " esp_wifi_set_ps = ");Serial_ESP_to_USB.println(esp_error); 
      WiFi.config(myIP, network_gateway, network_subnet);
      wifiConnected = true;
      setLedSequence(ledSeq_connected);
      Serial_ESP_to_USB.print(MSG_HEADER " CONNECTED! ");
      Serial_ESP_to_USB.print(MSG_HEADER " IP address: ");
      Serial_ESP_to_USB.println(WiFi.localIP());
    }
  } else {
    // Disconnected from AP
    if (wifiConnected) {
      // Transition Connected => Disconnected
      wifiConnected = false;
      setLedSequence(ledSeq_connecting);
      Serial_ESP_to_USB.print(MSG_HEADER " DISCONNECTED");
      disconnectPFODClient();
      connectCnt = 0;
    }
    Serial_ESP_to_USB.print(MSG_HEADER " Connecting ...");
    Serial_ESP_to_USB.println(connectCnt);
    connectCnt++;
    delay(250);
    if (connectCnt >= 20) {
        connectCnt = 0;
        Serial_ESP_to_USB.println();
        Serial_ESP_to_USB.println(MSG_HEADER " Wifi Reconnect");
        WiFi.reconnect();
        WiFi.setHostname("TeensyMower");  
        Serial_ESP_to_USB.println(); 
        esp_err_t esp_error = esp_wifi_set_ps(WIFI_PS_NONE);Serial_ESP_to_USB.print(MSG_HEADER " esp_wifi_set_ps = ");Serial_ESP_to_USB.println(esp_error);
        delay(250);
    }        
  }
}
