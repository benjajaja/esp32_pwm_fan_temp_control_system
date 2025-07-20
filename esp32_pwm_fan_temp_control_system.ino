// ============================================
// ESP32 Fan Control System for PC, Server, and Rack Temperature Monitoring
// Author: Claude Wolter
// License: Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
// Target Board: ESP32
// Compilation: Tested with ESP32 Arduino Core version 3.0.7
//
// Description:
// This project is an open-source system designed to control multiple 12V PWM fans based on temperature readings from DS18B20 sensors.
// It uses an ESP32 microcontroller and supports MQTT over Wi-Fi to report sensor data and fan speeds. The system ensures continuous
// operation even when Wi-Fi or MQTT connections fail.
//
// WARNING
// The ESP32 is not 5V tolerant. Ensure that all connected components operate within the ESP32's voltage limits (typically 3.3V)
// to avoid damage.
//
// NOTES
// The ESP8266 is not suitable for fans requiring high PWM frequencies above 1kHz, as it lacks support for higher PWM rates. 
// This sketch is not compatible with ESP8266 due to the use of the ESP32-specific LEDC API for PWM control.
// If DS18B20 show -127°C, they are either not connected to the right pin or a 5kΩ pullup is needed on the pin.
//
// ============================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================
// USER CONFIGURATION SECTION
// ============================

const char* mqtt_status_topic = (String(HASS_DEVICE_ID) + "/status").c_str();
const char* mqtt_control_topic = (String(HASS_DEVICE_ID) + "/control").c_str();
const char* mqtt_client_name = (String("ESP32Client_") + HASS_DEVICE_ID).c_str();

// Sensor and Fan Configuration
const int NUM_SENSORS = 1;    // Number of DS18B20 temperature sensors
const int NUM_FANS = 1;       // Number of fans

// Temperature Control Parameters
const float MIN_TEMP = 35.0; // Minimum temperature for lowest fan speed
const float MAX_TEMP = 55.0; // Maximum temperature for full fan speed

// Fan Speed Control
const int MIN_FAN_SPEED = 50;  // Minimum fan speed (PWM value, 0-255)
const int MAX_FAN_SPEED = 255; // Maximum fan speed (PWM value, 0-255)

// PWM Frequency
const int PWM_FREQUENCY = 25000; // Frequency for the PWM signal in Hz. Adjust this based on fan requirements.

// Timers (milliseconds)
const unsigned long reportInterval = 60000;
const unsigned long reconnectInterval = 600000;
const int reconnectAttempts = 3;

// ============================

// Pin Definitions
#define FAN_PWM 12         // GPIO12 (D6) - PWM control for all fans
#define ONE_WIRE_BUS 5     // GPIO4 (D2) - Dallas temperature sensors data pin

// LEDC PWM Channel
#define FAN_CHANNEL 0      // PWM channel for all fans

WiFiClient espClient;
PubSubClient client(espClient);

// Temperature Sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long lastReportTime = 0; // Timer for MQTT reporting
unsigned long lastReconnectAttempt = 0; // Timer for reconnect attempts

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  int attempts = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED && attempts < reconnectAttempts) {
    delay(5000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi. Proceeding offline.");
  }

  client.setBufferSize(512);
}

void wait_wifi() {
  Serial.println();
  Serial.print("Reconnecting to ");
  Serial.println(WIFI_SSID);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < reconnectAttempts) {
    delay(5000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi. Proceeding offline.");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message received: ");
  Serial.println(message);

  int pwmValue = message.toInt();
  pwmValue = constrain(pwmValue, MIN_FAN_SPEED, MAX_FAN_SPEED);

  // Apply PWM value to all fans
  ledcWrite(FAN_CHANNEL, pwmValue);
}

bool reconnect() {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect to the MQTT broker
    if (client.connect(mqtt_client_name)) {
      Serial.println("MQTT connected successfully.");
      client.subscribe(mqtt_control_topic); // Subscribe to the desired topic
      Serial.print("Subscribed to topic: ");
      Serial.println(mqtt_control_topic);
      reportSelf();
      return true;
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in the next loop.");
      return false;
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  // Initialize temperature sensors
  sensors.begin();

  reconnect();
  // Activate internal pullup if supported
  //pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  // Configure PWM pin
  if (!ledcAttachChannel(FAN_PWM, PWM_FREQUENCY, 8, FAN_CHANNEL)) {
    Serial.println("Failed to configure PWM channel");
    while (true); // Halt if configuration fails
  }
}

void reportSelf() {
  String payload =
  "{"
  "  \"name\": \"" + String(HASS_DEVICE_NAME) + "\","
  "  \"state_topic\": \"" + String(mqtt_status_topic) +"\","
  "  \"unique_id\": \"" + String(HASS_DEVICE_ID) +"\","
  "  \"unit_of_measurement\": \"°C\","
  "  \"value_template\": \"{{ value_json.temperature }}\","
  "  \"device_class\": \"temperature\""
  "}";
  String topic = "homeassistant/sensor/" + String(HASS_DEVICE_ID) + "/config";
  if (client.publish(topic.c_str(), payload.c_str(), true)) {
    Serial.println("Reported self to " + String("homeassistant/sensor/") + String(HASS_DEVICE_ID) + "/config");
  } else {
    Serial.println("MQTT publish failed!");
  }
  Serial.println(payload.c_str());
}

void reportStatus(float maxTemp, int pwmValue) {
  if (maxTemp > 50.0) {
    Serial.println("not reporting a temperatue above 50.0");
    return;
  }
  String payload = "{";
  payload += "\"temperature\":" + String(maxTemp);
  payload += ", \"pwm\":" + String(pwmValue);
  payload += "}";
  client.publish(mqtt_status_topic, payload.c_str());
  Serial.println("Published to MQTT.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    wait_wifi();
  }
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected() && millis() - lastReconnectAttempt > reconnectInterval) {
      lastReconnectAttempt = millis();
      reconnect();
    }
    client.loop();
  }

  // Read temperatures and control fans regardless of Wi-Fi connection
  sensors.requestTemperatures();
  float maxTemp = -127.0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    float temp = sensors.getTempCByIndex(i);
    if (temp > maxTemp) {
      maxTemp = temp;
    }
  }

  int pwmValue = (maxTemp <= MIN_TEMP) ? MIN_FAN_SPEED : (maxTemp >= MAX_TEMP) ? MAX_FAN_SPEED : map(maxTemp, MIN_TEMP, MAX_TEMP, MIN_FAN_SPEED, MAX_FAN_SPEED);
  ledcWrite(FAN_CHANNEL, pwmValue);

  if (WiFi.status() == WL_CONNECTED && millis() - lastReportTime > reportInterval) {
    lastReportTime = millis();
    reportStatus(maxTemp, pwmValue);
  }

  // Always print temperature and PWM for debugging
  Serial.print("Max Temp: ");
  Serial.print(maxTemp);
  Serial.print(" °C, PWM: ");
  Serial.println(pwmValue);

  delay(1000);
}
