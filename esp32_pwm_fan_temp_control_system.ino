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

static const char mqtt_status_topic[] = HASS_DEVICE_ID "/status";
static const char mqtt_control_topic[] = HASS_DEVICE_ID "/set";
static const char mqtt_client_name[] = "ESP32Client_" HASS_DEVICE_ID;

// Sensor and Fan Configuration
const int NUM_SENSORS = 1;    // Number of DS18B20 temperature sensors
const int NUM_FANS = 1;       // Number of fans

// Temperature Control Parameters
const float MIN_TEMP = 35.0; // Minimum temperature for lowest fan speed
const float MAX_TEMP = 55.0; // Maximum temperature for full fan speed

// Fan Speed Control
const int MIN_FAN_SPEED = 96;  // Minimum fan speed (PWM value, 0-255)
const int MAX_FAN_SPEED = 255; // Maximum fan speed (PWM value, 0-255)

// Temperature thresholds
const float FAN_ON_TEMP = 25.0;

// PWM Frequency
const int PWM_FREQUENCY = 25000; // Frequency for the PWM signal in Hz. Adjust this based on fan requirements.

// Timers (milliseconds)
const unsigned long reportInterval = 2000;
const unsigned long reconnectInterval = 5000;
const int reconnectAttempts = 3;

// ============================

// Pin Definitions
#define FAN_PWM 12         // GPIO12 (D6) - PWM control for all fans
#define ONE_WIRE_BUS 5     // GPIO4 (D2) - Dallas temperature sensors data pin

// LEDC PWM Channel
#define FAN_CHANNEL 0      // PWM channel for all fans

const int TACH_PIN = 13;
const int SAMPLE_TIME = 1000;

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
    Serial.print("WiFi connected, ");
    Serial.print("IP address: ");
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

static int overridePwm = -1.0;

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message received on ");
  Serial.print(topic);
  Serial.println(":");
  Serial.println(message);

  int pwmValue = 0;
  if (strcmp(message.c_str(), "ON") == 0) {
    pwmValue = MAX_FAN_SPEED;
  } else if (strcmp(message.c_str(), "OFF") == 0) {
    pwmValue = 0;
  } else {
    int pwmPct = message.toInt();
    pwmValue = (pwmPct * MAX_FAN_SPEED) / 100;
  }
  pwmValue = constrain(pwmValue, 0.0, MAX_FAN_SPEED);

  overridePwm = pwmValue;
  // Apply PWM value to all fans
  Serial.print("Set fan to ");
  Serial.println(pwmValue);
  // ledcWrite(FAN_CHANNEL, pwmValue);
  analogWrite(FAN_PWM, pwmValue);
}

bool reconnect() {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection as ");
    Serial.print(mqtt_client_name);
    Serial.println("...");

    // Attempt to connect to the MQTT broker
    if (client.connect(mqtt_client_name)) {
      Serial.println("MQTT connected successfully.");
      client.subscribe(mqtt_control_topic); // Subscribe to the desired topic
      Serial.print("Subscribed to topic: ");
      Serial.println(mqtt_control_topic);
      registerSensor("temperature", "°C");
      registerSensor("pwm", "%");
      registerSensor("rpm", "rpm");
      registerNumber();
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
  // ledcSetup(FAN_CHANNEL, PWM_FREQUENCY, 8);
  // ledcAttachPin(FAN_PWM, FAN_CHANNEL);
  // if (!ledcAttachChannel(FAN_PWM, PWM_FREQUENCY, 8, FAN_CHANNEL)) {
    // Serial.println("Failed to configure PWM channel");
    // while (true); // Halt if configuration fails
  // }
  pinMode(FAN_PWM, OUTPUT);
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), countPulse, FALLING);
}

volatile unsigned int pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;
void countPulse() {
  pulseCount++;
}

void measureRPM() {
  // Reset pulse counter
  pulseCount = 0;
  lastTime = millis();
  
  // Wait for sample period
  delay(SAMPLE_TIME);
  
  // Calculate RPM
  // Most fans generate 2 pulses per revolution
  // RPM = (pulses / 2) * (60000 / sample_time_ms)
  rpm = (pulseCount / 2.0) * (60000.0 / SAMPLE_TIME);
}

void registerSensor(char* value, char* unit_of_measurement) {
  char topic[128];
  char payload[512];
  
  // Build topic string
  char entity[64];
  strcpy(
      entity, 
      strcmp(value, "temperature") == 0 || strcmp(value, "rpm") == 0 
        ? "sensor" 
        : "fan"
  );
  snprintf(topic, sizeof(topic), "homeassistant/%s/%s_%s/config", entity, HASS_DEVICE_ID, value);
  
  // Build payload - common part
  int pos = snprintf(payload, sizeof(payload),
    "{"
    "\"name\":\"%s %s\","
    "\"state_topic\":\"%s\","
    "\"unique_id\":\"%s_%s\","
    "\"unit_of_measurement\":\"%s\","
    "\"value_template\":\"{{ value_json.%s }}\"",
    HASS_DEVICE_NAME, value, 
    mqtt_status_topic, 
    HASS_DEVICE_ID,
    value,
    unit_of_measurement,
    value);
  
  // Add conditional part
  if (strcmp(value, "temperature") == 0) {
    pos += snprintf(payload + pos, sizeof(payload) - pos,
      ",\"device_class\":\"%s\","
      "\"device\":{"
      "\"identifiers\":[\"%s\"],"
      "\"name\":\"%s\","
      "\"model\":\"ESP32 Sensor\","
      "\"manufacturer\":\"Benja\""
      "}}", value, HASS_DEVICE_ID, HASS_DEVICE_NAME);
  } else {
    pos += snprintf(payload + pos, sizeof(payload) - pos,
      "\"device\":{"
      "\"identifiers\":[\"%s\"]"
      "}}", HASS_DEVICE_ID);
  }
  
  if (client.publish(topic, payload, true)) {
    Serial.print("Reported self to ");
    Serial.println(topic);
    Serial.println(payload);
  } else {
    Serial.println("MQTT publish failed!");
  }
}

void registerNumber() {
  char topic[128];
  char payload[512];
  
  // Build topic string
  snprintf(topic, sizeof(topic), "homeassistant/number/%s_pwm/config", HASS_DEVICE_ID);
  
  // Build payload - common part
  snprintf(payload, sizeof(payload),
    "{"
    "\"name\":\"%s pwm control\","
    "\"command_topic\":\"%s\","
    "\"unique_id\":\"%s_pwm_control\","
    "\"device\":{"
    "\"identifiers\":[\"%s\"]"
    "}}", 
    HASS_DEVICE_NAME,
    mqtt_control_topic, 
    HASS_DEVICE_ID,
    HASS_DEVICE_ID);
  
  if (client.publish(topic, payload, true)) {
    Serial.print("Reported self to ");
    Serial.println(topic);
    Serial.println(payload);
  } else {
    Serial.println("MQTT publish failed!");
  }
}

void reportStatus(float maxTemp, int pwmValue, int rpmValue) {
  if (maxTemp > 50.0 || maxTemp < 0.0) {
    Serial.println("Not reporting a temperatue above 50.0 or below 0.0");
    return;
  }
  
  char payload[64];  // Small buffer for simple JSON
  
  // Build JSON payload with proper float formatting
  int pwmPct = pwmValue * 100 / MAX_FAN_SPEED;
  snprintf(
      payload,
      sizeof(payload),
      "{\"temperature\":%.1f,\"pwm\":%d,\"rpm\":%d}",
      maxTemp,
      pwmPct,
      rpmValue
  );
  
  if (client.publish(mqtt_status_topic, payload)) {
    Serial.print(mqtt_status_topic);
    Serial.print(" ");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish to MQTT.");
  }
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
  float temp = -127.0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    float ctemp = sensors.getTempCByIndex(i);
    if (ctemp > temp) {
      temp = ctemp;
    }
  }

  static bool fanRunning = false;
  if (temp >= FAN_ON_TEMP) {
    fanRunning = true;
  } else if (temp <= FAN_ON_TEMP - 2.0) {
    fanRunning = false;
  }
  int pwmValue = 0;
  if (fanRunning) {
    // Map temperature to PWM range (30°C = min speed, higher = faster)
    // Adjust the upper temperature limit as needed
    pwmValue = map(constrain(temp * 100, 3000, 5000), 
                   3000, 5000,           // 30.0°C to 50.0°C 
                   MIN_FAN_SPEED, MAX_FAN_SPEED);
  } else {
    pwmValue = 0;  // Fan completely off
  }
  if (overridePwm != -1.0) {
    pwmValue = overridePwm;
  }
  // ledcWrite(FAN_CHANNEL, pwmValue);
  analogWrite(FAN_PWM, pwmValue);

  if (WiFi.status() == WL_CONNECTED && millis() - lastReportTime > reportInterval) {
    lastReportTime = millis();
    reportStatus(temp, pwmValue, rpm);
  }

  // Always print temperature and PWM for debugging
  Serial.print("(loop) Temp: ");
  Serial.print(temp);
  Serial.print(" °C, PWM: ");
  Serial.println(pwmValue);

  Serial.print("RPM: ");
  Serial.println(rpm);

  measureRPM();
}
