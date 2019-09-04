#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

// Declare update_encoder as ISR
void ICACHE_RAM_ATTR update_encoder();

// Motor pins
const char motor1Pin1 = D7;
const char motor1Pin2 = D6;
const char enable1Pin = D8; 

// Encoder pin
const char encoderPin = D5;
volatile long encoder_value = 0;

// Timers to monitor motor stall
// these are whats fucking up in if condition in encoder_count fn
volatile unsigned long current_time;
volatile unsigned long previous_time;
unsigned long stall_time = 100; // 100ms 

// Window status
char* window_status;

// Setup mqtt client
WiFiClient wifiClient;
PubSubClient client(mqttServer, mqttPort, wifiClient);

void setup() {
	// Start serial comms
	Serial.begin(115200);

	// Set up pins
	pinMode(motor1Pin1, OUTPUT);
	pinMode(motor1Pin2, OUTPUT);
	pinMode(enable1Pin, OUTPUT);
	pinMode(encoderPin, INPUT_PULLUP);

	// Set encoder interrupt to pin
	attachInterrupt(digitalPinToInterrupt(encoderPin), update_encoder, FALLING);

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

/* 
Function to read encoder
Checks encoder value is less than the desired number of motor rotations
Also checks time since last encoder tick, to make sure motor hasn't stalled
This function is essentially just a while loop that does nothing, as when this
function ends, the next function to be called is motor_stop
*/
void encoder_count(float rotations) {
  rotations = 150*11*rotations; // 150:1 gearbox reduction and 11 ticks per rotation
	while (encoder_value < rotations) {
   
   // Motor stall monitoring
   if (encoder_value > 50) { // make sure the motor has spun up to speed before we check if its stalled
    	current_time = millis();
      // Using timer variable to fix overflow problem of 32-bit unsigned long
      // int overflows more often, but returns -1 when it does, so just added 2
      // ensures its always positive
      unsigned int timer = current_time - previous_time + 2;
      /* 
      Check if motor has stalled by making sure time between encoder ticks
      doesnt exceed global stall_time variable (currently set to 100ms)
      */
      if (timer >= stall_time) {
        Serial.println("Motor stalled");
        Serial.println(timer);
        break;
    	}
    }
    ESP.wdtFeed(); // Feed the watchdog so it doesnt reboot the controller
	}
}

// ISR to increment encoder value (input to encoder_count)
void update_encoder() {
	encoder_value++;
  previous_time = millis();
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
    	encoder_count(rotations + 2); // +2 rotations to make sure it closes tight (will stall motor) 
    	motor_stop();
    	Serial.println("Motor stopped");
    }
  }
}

// Main arduino loop, just runs MQTT loop, everthing else is interrupt driven
void loop() {
	client.loop();
}
