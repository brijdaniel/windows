#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "Motor.h"
#include "Encoder.h"

// Motor pins and speed
const char motor1Pin1 = D7;
const char motor1Pin2 = D6;
const char enable1Pin = D8; 
int speed = 1000; // full speed

// Create motor object(s)
Motor motor1(enable1Pin, motor1Pin1, motor1Pin2, speed);

// Encoder pin
const char encoder1Pin = D5;

// Create encoder object(s)
Encoder encoder1(encoder1Pin);

// Declare update_encoder as ISR
void ICACHE_RAM_ATTR encoder1.update_encoder();

// Declare Window status variable
char* window_status;

// Setup mqtt client
WiFiClient wifiClient;
PubSubClient client(mqttServer, mqttPort, wifiClient);

void setup() {
	// Start serial comms
	Serial.begin(115200);

	// Set encoder interrupt to pin
	attachInterrupt(digitalPinToInterrupt(encoder1Pin), encoder1.update_encoder, FALLING);

	// Connect to wifi
	Serial.print("Connecting to ");
  	Serial.println(ssid);
	WiFi.begin(ssid, wifiPassword);

	while (WiFi.status() != WL_CONNECTED) {
	  delay(500);
	  Serial.print(".");
	}

	// Print IP Address of the ESP8266
	Serial.println("WiFi connected");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	// Connect to MQTT Broker
	client.setServer(mqttServer, mqttPort);
	client.setCallback(callback);

	// Check if connection was successful
	if (client.connect(clientID)) {
	  Serial.println("Connected to MQTT Broker!");
	}
	else {
	  Serial.println("Connection to MQTT Broker failed...");
	}
  	// Subscribe to mqtt topics and publish request for window status from server database
	client.subscribe("pihouse/windows/control");
	client.subscribe("pihouse/windows/status");
	client.publish("pihouse/windows/status", "request");
}

// MQTT callback function, executed when a message is received
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  // if topic is window status then store and print window_status
  if (strcmp(topic, "pihouse/windows/status") == 0) {
  	payload[length] = '\0';
  	window_status = (char*)payload; 
    Serial.println(window_status); 
  }
  
  /* 
  if topic is control, ie command to open or close window, then payload will be json
  containing direction (ie open or close) and how many rotations of the motor (input to encoder_count)
  Regardless of if direction is open or close, logic is as follows:
  - Store payload data into direction and rotations variables
  - Check the window isnt already in the state we are trying to drive it to
  - Publish that the state has changed (this is pre-emptive)
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
      client.publish("pihouse/windows/status", "open");
  	  motor1.m_fwd();
  	  encoder1.encoder_count(rotations); 
  	  motor1.m_stop();
  	  Serial.println("Motor stopped");
    } else if (direction == "close" && window_status != "closed") {
    	Serial.println("Motor Backwards"); 
    	client.publish("pihouse/windows/status", "closed");
      motor1.m_back();
    	encoder1.encoder_count(rotations + 2); // +2 rotations to make sure it closes tight (will stall motor) 
    	motor1.m_stop();
    	Serial.println("Motor stopped");
    }
  }
}

// Main arduino loop, just runs MQTT loop, everthing else is interrupt driven
void loop() {
	client.loop();
}
