#include <ESP8266WiFi.h>
#include "wifiConnection.h"
#include "config.h"

void connect_wifi(void){
	// Connect to wifi
	Serial.print("Connecting to ");
  Serial.println(ssid);

  // Explicitly set it to client only, otherwise
  // by default it acts as both wifi client and 
  // access point
  WiFi.mode(WIFI_STA);
	
	if (WiFi.SSID() != ssid) {
	  WiFi.begin(ssid, wifiPassword);
    WiFi.persistent(true);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
	};
 
	while (WiFi.status() != WL_CONNECTED) {
	  delay(500);
	  Serial.print(".");
	};

	// Print IP Address of the ESP8266
	Serial.println("WiFi connected");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
};
