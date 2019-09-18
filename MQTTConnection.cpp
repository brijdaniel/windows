#include <PubSubClient.h>
#include "MQTTConnection.h"
#include "config.h"

void connect_MQTT(PubSubClient client){
	// Connect to MQTT Broker
	client.setServer(mqttServer, mqttPort);

	// Check if connection was successful
	if (client.connect(clientID)) {
	  Serial.println("Connected to MQTT Broker!");
	} else {
	  Serial.println("Connection to MQTT Broker failed...");
	};
	
  	// Subscribe to mqtt topics and publish request for window status from server database
	client.subscribe(controlChannel);
	client.subscribe(statusChannel);
	client.publish("pihouse/windows/status", "request");
};
