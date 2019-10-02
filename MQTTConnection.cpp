#include <PubSubClient.h>
#include "MQTTConnection.h"
#include "config.h"

void connect_MQTT(PubSubClient client){
	// Connect to MQTT Broker
	client.setServer(mqttServer, mqttPort);

	// Check if connection was successful
	if (client.connect(clientID)) {
	  Serial.print("Connected to MQTT Broker with status ");
    Serial.println(client.state());
	} else {
	  Serial.println("Connection to MQTT Broker failed...");
	};
	
  	// Subscribe to mqtt topics and publish request for window status from server database
	client.subscribe(controlChannel);
	client.subscribe(requestChannel);
	client.publish(statusChannel, "request");
};

boolean reconnect_MQTT(PubSubClient client) {
  if (client.connect(clientID)) {
    Serial.print("Reconnected successfully with status ");
    Serial.println(client.state());
    client.subscribe(controlChannel);
    client.subscribe(requestChannel);
    client.publish(statusChannel, "request");
  }
  return client.connected();
}
