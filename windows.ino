#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

void ICACHE_RAM_ATTR update_encoder();

// Motor pins
const char motor1Pin1 = D7;
const char motor1Pin2 = D6;
const char enable1Pin = D8; 

// Encoder pin
const char encoderPin = D5;
volatile long encoder_value = 0;

// Encoder interrupt timer
unsigned long timer;

// Flag to make sure motor starts before timer is checked
bool flag = false;

// Window status
char* window_status;

// setup mqtt client
WiFiClient wifiClient;
PubSubClient client(mqttServer, mqttPort, wifiClient);

void setup() {
	// start serial
	Serial.begin(115200);

	// set up pins
	pinMode(motor1Pin1, OUTPUT);
	pinMode(motor1Pin2, OUTPUT);
	pinMode(enable1Pin, OUTPUT);
	pinMode(encoderPin, INPUT_PULLUP);

	// Interrupt for encoder
	attachInterrupt(digitalPinToInterrupt(encoderPin), update_encoder, FALLING);

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
	client.subscribe("pihouse/windows/control");
	client.subscribe("pihouse/windows/status");
	client.publish("pihouse/windows/status", "request"); // not sure if I'll receive reply bcos loop hasnt started yet
}

// functions to drive motor
void motor_fwd(void) {
	analogWrite(enable1Pin, 1000);
	digitalWrite(motor1Pin1, LOW);
	digitalWrite(motor1Pin2, HIGH); 
}

void motor_back(void) {
	analogWrite(enable1Pin, 1000);
	digitalWrite(motor1Pin1, HIGH);
	digitalWrite(motor1Pin2, LOW); 
}

void motor_stop(void) {
	// Motor braking
	analogWrite(enable1Pin, 1000);
	digitalWrite(motor1Pin1, HIGH);
	digitalWrite(motor1Pin2, HIGH);
  	delay(200);
  
  // When motor is stopped, turn off all pins
  analogWrite(enable1Pin, 0);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
}

// function to read encoder
void encoder_count(float rotations) {
  rotations = 150*11*rotations; // 150:1 gearbox reduction and 11 ticks per rotation
	while (encoder_value < rotations) {
   /* if (flag) {
    	if (millis()-timer > 100) {
        break;
    	}
      //Serial.println(millis()-timer);
    } */
    ESP.wdtFeed();
	}
  // Reset timer flag
  flag = false;
}

void update_encoder() {
	encoder_value++;
  flag = true;
  timer = millis();
}

// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  if (strcmp(topic, "pihouse/windows/status") == 0) {
  	payload[length] = '\0';
  	window_status = (char*)payload; 
    Serial.println(window_status); 
  }
  
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
        encoder_value = 0;
  	  motor_fwd();
  	  encoder_count(rotations); 
  	  motor_stop();
  	  Serial.println("Motor stopped");
    } else if (direction == "close" && window_status != "closed") {
    	Serial.println("Motor Backwards"); 
    	client.publish("pihouse/windows/status", "closed");
    	encoder_value = 0;
      motor_back();
    	encoder_count(rotations); 
    	motor_stop();
    	Serial.println("Motor stopped");
    }
  }
}

void loop() {
	client.loop();
}
