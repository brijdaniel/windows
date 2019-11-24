
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "Motor.h"
#include "Encoder.h"
#include "wifiConnection.h"
#include "MQTTConnection.h"

// Motor pins and speed
const char motor1Pin1 = D7;
const char motor1Pin2 = D6;
const char enable1Pin = D8; 
int speed = 1000; // full speed

// Encoder pin
const char encoder1Pin = D5;

// Create motor object(s)
Motor motor1(enable1Pin, motor1Pin1, motor1Pin2, speed);

// Encoder interrupt forward dispatch for ISR
// Must be put into RAM as it is part of the ISR function tree
void ICACHE_RAM_ATTR encoder_dispatch();

// Create encoder object(s), include interrupt dispatch
Encoder encoder1(encoder1Pin, &encoder_dispatch);

// Declare Window status variable
char* window_status;

// Declare counter for MQTT reconnection
long lastReconnectAttempt = 0;

// Setup mqtt client
WiFiClient wifiClient;
PubSubClient client(mqttServer, mqttPort, wifiClient);

void setup() {
	// Start serial comms, connect to Wifi and MQTT
	Serial.begin(115200);
	connect_wifi();
  client.setCallback(callback);
	connect_MQTT(client);
};

// MQTT callback function, executed when a message is received
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  // if message is a response to a status request, then store and print window_status
  if (strcmp(topic, "pihouse/windows/status") == 0) {
  	payload[length] = '\0';
  	window_status = (char*)payload; 
    window_status[length] = '\0'; // terminate str
    Serial.println(window_status); 
  };
  
  /* 
  if topic is control, ie command to open or close window, then payload will be json
  containing direction (ie open or close) and how many rotations of the motor (input to encoder_count)
  Regardless of if direction is open or close, logic is as follows:
  - Store payload data into direction and rotations variables
  - Check the window isnt already in the state we are trying to drive it to
  - Publish that the state has changed (this is pre-emptive). Set this as a retained msg so ESP receives this status again when it reconnects to broker
  - Set global encoder_value var to 0 (used in the encoder_count fn and incremented by update_encoder ISR)
  - Call fn to start driving motor either fwds or back
  - Start monitoring encoder ticks with encoder_count (this while loop is now blocking, but still getting interrupts from ISR)
  - When while loop expires, ie when number of rotations is reached, move to next line which is motor_stop
  */
  if (strcmp(topic, "pihouse/windows/control") == 0) {
    // handle JSON payload
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    const String direction = doc["direction"];
    const float rotations = doc["rotations"];
    Serial.println(direction);
    Serial.println(rotations);
    
    if (direction == "open" && window_status != "open") {
  	  Serial.println("Motor Forward");
      client.publish(statusChannel, "open", true);
  	  motor1.m_fwd();
  	  encoder1.encoder_count(rotations); 
  	  motor1.m_stop();
      client.publish(connectedChannel, "False");
  	  Serial.println("Motor stopped");
      delay(0.5);
      ESP.deepSleep(0); // Put ESP to sleep after it has woken up and completed task
    } else if (direction == "close" && window_status != "closed") {
    	Serial.println("Motor Backwards"); 
    	client.publish(statusChannel, "closed", true);
      motor1.m_back();
    	encoder1.encoder_count(rotations + 2); // +2 rotations to make sure it closes tight (will stall motor) 
    	motor1.m_stop();
      client.publish(connectedChannel, "False");
    	Serial.println("Motor stopped");
      delay(0.5);
      ESP.deepSleep(0); // Put ESP to sleep after it has woken up and completed task
    };
  };
};

/* 
Main loop: 
 - Check connection to MQTT server
 - If connection has failed, then fully disconnect, and continue trying to reconnect
   every 0.5 secs
 - Else, run MQTT loop
*/
void loop() {
	/*
   Modified the PubSubClient source code to fix MQTT_CONNECTION_TIMEOUT failure
   Failure was result of calculation using UL, which appears to cause problems with ESP
	 */
  if (!client.connected()) {
    long now = millis();
    Serial.println(client.state());
    client.publish(connectedChannel, "False");
    client.disconnect();
    
    if (now - lastReconnectAttempt > 500) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect_MQTT(client)) {
        lastReconnectAttempt = 0;
      };
    };
  } else {
    client.loop();
  };

  // Put ESP into deep sleep 10s after it boots up, or if wakes up and doesnt receive a control msg
  if (millis() >= 10000) {
    client.publish(connectedChannel, "False");
    delay(0.5);
    ESP.deepSleep(0);
  };
};

void encoder_dispatch() {
	encoder1.update_encoder();
};
