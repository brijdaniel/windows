#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

// Motor pins
const char motor1Pin1 = D7;
const char motor1Pin2 = D6;
const char enable1Pin = D8; 

// Encoder pin
const char encoderPin = D5;

// PWM properties
/*
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;
*/

// setup mqtt client
WiFiClient wifiClient;
PubSubClient client(mqttServer, mqttPort, wifiClient);


void setup() {
	// set up pins
	pinMode(motor1Pin1, OUTPUT);
	pinMode(motor1Pin2, OUTPUT);
	pinMode(enable1Pin, OUTPUT);
	pinMode(encoderPin, INPUT);

	// start serial
	Serial.begin(115200);

	// connect to wifi
	Serial.print("Connecting to ");
  	Serial.println(ssid);
	WiFi.begin(ssid, wifiPassword);

	while (WiFi.status() != WL_CONNECTED) {
	  delay(500);
	  Serial.print(".");
	}

	// Debugging - Output the IP Address of the ESP8266
	Serial.println("WiFi connected");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	// Connect to MQTT Broker
	client.setServer(mqttServer, mqttPort);
	client.setCallback(callback);

	// client.connect returns a bool value to let us know if the connection was successful.
	if (client.connect(clientID)) {
	  Serial.println("Connected to MQTT Broker!");
	}
	else {
	  Serial.println("Connection to MQTT Broker failed...");
	}
	client.subscribe("pihouse/windows/#");
}

// functions to drive motor
void motor_fwd(void) {
	analogWrite(enable1Pin, 2000);
	digitalWrite(motor1Pin1, LOW);
	digitalWrite(motor1Pin2, HIGH); 
	Serial.println("Motor Forward");
}

void motor_back(void) {
	analogWrite(enable1Pin, 1000);
	digitalWrite(motor1Pin1, HIGH);
	digitalWrite(motor1Pin2, LOW); 
	Serial.println("Motor Backwards"); 
}

void motor_stop(void) {
	analogWrite(enable1Pin, 0);
	digitalWrite(motor1Pin1, LOW);
	digitalWrite(motor1Pin2, LOW);
	Serial.println("Motor stopped");
}

// function to read encoder
void encoder_count(float rotations) {
	int i = 0;
	int state = 0;
	int prev_state = 0;
  rotations = (145*20)*rotations; // 150:1 gearbox reduction and ~21? ticks per rotation
	while (i < rotations) {
		state = digitalRead(encoderPin);
		if (state != prev_state) { // pretty sure this will count two ticks per actual tick
			i++;
			//Serial.println(i); 
		}
		prev_state = state;
    ESP.wdtFeed();
	}
}

// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  /*
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  byte msg_byte = *payload;
  Serial.println(msg_byte);
  int msg = (int) msg_byte - '0'; // minus 0 to convert ascii to int
  Serial.println(msg);
  */
  
  // handle JSON payload
  StaticJsonDocument<256> doc;
  deserializeJson(doc, payload, length);
  const String direction = doc["direction"];
  const float rotations = doc["rotations"];
  Serial.println(direction);
  Serial.println(rotations);
  
  if (direction == "fwd") {
	  motor_fwd();
	  encoder_count(rotations); 
	  motor_stop();
  } else if (direction == "back") {
  	motor_back();
  	encoder_count(rotations); 
  	motor_stop();
  }
}

void loop() {
	client.loop();
}
