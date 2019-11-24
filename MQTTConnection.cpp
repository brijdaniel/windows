#include <PubSubClient.h>
#include "MQTTConnection.h"
#include "config.h"

void connect_MQTT(PubSubClient client){
	// Connect to MQTT Broker
	client.setServer(mqttServer, mqttPort);

	// Connect to MQTT broker
	// LWT to publish on connectedChannel "False", this tells the server that the ESP is no longer connected to the MQTT broker 
	if (client.connect(clientID, NULL, NULL, connectedChannel, 2, 0, "False")) {
	  Serial.print("Connected to MQTT Broker with status ");
    Serial.println(client.state());
	} else {
	  Serial.println("Connection to MQTT Broker failed...");
	};
	
  	// Subscribe to mqtt topics
	client.subscribe(controlChannel, 1);
    client.subscribe(statusChannel, 1);
    client.subscribe(connectedChannel);

    // Publish message telling the server that the ESP is connected to the MQTT broker
	client.publish(connectedChannel, "True");
};

// Handle reconnection and resubscription if connection drops
boolean reconnect_MQTT(PubSubClient client) {
  if (client.connect(clientID, NULL, NULL, connectedChannel, 2, 0, "False")) {
    Serial.print("Reconnected successfully with status ");
    Serial.println(client.state());
    client.subscribe(controlChannel, 1);
    client.subscribe(statusChannel, 1);
	client.publish(connectedChannel, "True");  }
  return client.connected();
}
