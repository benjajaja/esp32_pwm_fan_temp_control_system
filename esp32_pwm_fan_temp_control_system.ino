// ============================================
// ESP32 Fan Control System for PC, Server, and Rack Temperature Monitoring
// Author: Claude Wolter
// License: Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
// Target Board: ESP32
// Compilation: Tested with ESP32 Arduino Core version 3.0.7
//
// Description:
// This project is an open-source system designed to control multiple PWM fans based on temperature readings from DS18B20 sensors.
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
// A DS18B20 may show -85.00°C on the first readings, it is ignored for accurate reporting.
//
// ============================================

#include "esp_bt.h"
#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// =============================
// Env vars (from .env.<device>)
// =============================
// WIFI_SSID: ssid (string)
// WIFI_PASSWORD: password (string), will be baked into binary!
// MQTT_SERVER: server hostname or IP (string)
// MQTT_PORT: port (number)
// HASS_DEVICE_ID: device ID (string), e.g. cabinet_esp32
// HASS_DEVICE_NAME: device name (string), e.g. Esp32FanCabinet
// FAN_ENABLED: enable fan (bool), e.g. can be false to be just a temperature sensor

// ============================
// USER CONFIGURATION SECTION
// ============================
#define FAN_PWM 12 // GPIO12 (D12) - PWM control
#define TACH_PIN 13 // GPIO13 (D13) - RPM signal
#define RELAY_PIN 14 // GPIO14 (D14) - Optional relay or mosfet pin (or anything else)
#define ONE_WIRE_BUS 5 // GPIO4 (D2) - Dallas temperature DS18B20 sensor data pin (probably needs ~5kΩ pull-up resistor).

#define MIN_FAN_SPEED 32  // Minimum fan speed (PWM value, 0-255)
#define MAX_FAN_SPEED 255 // Maximum fan speed (PWM value, 0-255)

#define FAN_ON_TEMP 32.0 // At what point will the fan be turned on at all.
#define FAN_TEMP_MIN 20.0 // At what point PWM will be at minimim. Should be lower than FAN_ON_TEMP. This maps directly to MIN_FAN_SPEED.
#define FAN_TEMP_MAX 40.0 // At what point PWM will be at maximum. This maps directly to MAX_FAN_SPEED.
#define FAN_TEMP_HYSTERISIS 2.0 // Fan will not turn off until FAN_ON_TEMP - FAN_TEMP_HYSTERISIS.

#define RPM_SAMPLE_TIME 1000 // How long to sample RPM
#define RPM_REPORT_MAX 5000

#define RECONNECT_ATTEMPTS 5 // How many WiFi reconnect attempts per loop
#define RECONNECT_INTERVAL_MS 5000 // How long to wait between MQTT reconnects
#define REPORT_INTERVAL_MS 1000 // How long to wait between MQTT reports

static const char mqtt_status_topic[] = HASS_DEVICE_ID "/status";
static const char mqtt_control_topic_pwm[] = HASS_DEVICE_ID "/pwm/set";
static const char mqtt_control_topic_override[] = HASS_DEVICE_ID "/override/set";
static const char mqtt_control_topic_threshold[] = HASS_DEVICE_ID "/threshold/set";
static const char mqtt_control_topic_max_temp[] = HASS_DEVICE_ID "/max_temp/set";
static const char mqtt_client_name[] = "ESP32Client_" HASS_DEVICE_ID;

static float threshold = FAN_ON_TEMP;
static float maxTemp = FAN_TEMP_MAX;
static int pwmValue = 0;
static bool relayEnabled = false;

Preferences preferences;

WiFiClient espClient;
PubSubClient client(espClient);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long lastReportTime = 0; // Timer for MQTT reporting
unsigned long lastReconnectAttempt = 0; // Timer for reconnect attempts

// LED blink timer
hw_timer_t * timer = NULL;
volatile bool ledState = false;
void IRAM_ATTR onTimer() {
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
}

void startBlink(int ms) {
  timer = timerBegin(1000000); // 1MHz frequency
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, ms * 1000, true, 0); // ms, repeat, no delay
}

void stopBlink(bool high) {
  timerEnd(timer);
  digitalWrite(LED_BUILTIN, high);
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  int attempts = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED && attempts < RECONNECT_ATTEMPTS) {
    Serial.println("Retry wifi...");
    startBlink(100);
    delay(500);
    stopBlink(WiFi.status() == WL_CONNECTED ? HIGH : LOW);
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, ");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    WiFi.setTxPower(WIFI_POWER_15dBm);
    WiFi.mode(WIFI_STA);
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
  while (WiFi.status() != WL_CONNECTED && attempts < RECONNECT_ATTEMPTS) {
    startBlink(100);
    delay(500);
    stopBlink(WiFi.status() == WL_CONNECTED ? HIGH : LOW);
    delay(500);
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
  Serial.print(": ");
  Serial.println(message);

  if (FAN_ENABLED) {
    if (strcmp(topic, mqtt_control_topic_override) == 0) {
      if (strcmp(message.c_str(), "auto") == 0) {
        overridePwm = -1.0;
        Serial.println("Set mode to auto");
        // PWM will set itself based on temp in loop.
      } else if (strcmp(message.c_str(), "override") == 0) {
        overridePwm = pwmValue;
        Serial.println("Set mode to override");
      } else {
        Serial.println("Bad override message");
      }

    } else if (strcmp(topic, mqtt_control_topic_pwm) == 0) {
      int pwmValue = 0;
      int pwmPct = message.toInt();
      pwmValue = (pwmPct * MAX_FAN_SPEED) / 100;
      pwmValue = constrain(pwmValue, 0.0, MAX_FAN_SPEED);
      overridePwm = pwmValue;
      Serial.print("Set fan to ");
      Serial.println(overridePwm);

    } else if (strcmp(topic, mqtt_control_topic_threshold) == 0) {
      threshold = message.toFloat();
      preferences.putFloat("FAN_ON_TEMP", threshold);
      Serial.print("Set threshold to ");
      Serial.println(threshold);
    } else if (strcmp(topic, mqtt_control_topic_max_temp) == 0) {
      maxTemp = message.toFloat();
      preferences.putFloat("FAN_TEMP_MAX", maxTemp);
      Serial.print("Set FAN_TEMP_MAX to ");
      Serial.println(maxTemp);
    } else {
      Serial.println("UNKNOWN TOPIC!");
    }
  }
}

bool mqtt_reconnect() {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection as ");
    Serial.print(mqtt_client_name);
    Serial.println("...");
    startBlink(700);

    bool success = client.connect(mqtt_client_name);
    stopBlink(success ? HIGH : LOW);
    if (success) {
      Serial.println("MQTT connected successfully.");
      client.subscribe(mqtt_control_topic_pwm);
      client.subscribe(mqtt_control_topic_override);
      client.subscribe(mqtt_control_topic_threshold);
      client.subscribe(mqtt_control_topic_max_temp);
      Serial.print("Subscribed to topics");
      registerSensor("temperature", "°C");
      if (FAN_ENABLED) {
        registerSensor("rpm", "rpm");
        registerSensor("pwm_connected", "");
        registerSelect(mqtt_control_topic_override);
        registerNumber("pwm", mqtt_control_topic_pwm, 0, 100);
        registerNumber("threshold", mqtt_control_topic_threshold, 20, 50);
        registerNumber("max_temp", mqtt_control_topic_max_temp, 30, 80);
      }
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in the next loop.");
    }
    return success;
  }
  return true;
}

void setup() {
  if (FAN_ENABLED) {
#ifdef RELAY_PIN
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Turn off relay
#endif
    analogWrite(FAN_PWM, 0); // Set PWM to zero
    threshold = preferences.getFloat("FAN_ON_TEMP", FAN_ON_TEMP);
    maxTemp = preferences.getFloat("FAN_TEMP_MAX", FAN_TEMP_MAX);
  }
  setCpuFrequencyMhz(80);
  esp_bt_controller_disable();
  esp_sleep_enable_timer_wakeup(1 * 1000000);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  setup_wifi();

  client.setServer(MQTT_SERVER, MQTT_PORT);
  if (FAN_ENABLED) {
    client.setCallback(callback);
  }

  // Initialize temperature sensors
  sensors.begin();

  mqtt_reconnect();

  // Configure PWM pin and RPM
  if (FAN_ENABLED) {
    pinMode(FAN_PWM, OUTPUT);
    pinMode(TACH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TACH_PIN), countPulse, FALLING);
  }
}

volatile unsigned int pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;
void countPulse() {
  pulseCount++;
}

void measureRPM() {
  // Reset pulse counter
  noInterrupts();
  pulseCount = 0;
  unsigned long startTime = millis();
  interrupts();
  
  delay(RPM_SAMPLE_TIME);
  
  noInterrupts();
  unsigned long actualElapsed = millis() - startTime;
  unsigned int count = pulseCount;
  interrupts();
  
  // Calculate RPM using actual elapsed time
  // Most fans generate 2 pulses per revolution
  // RPM = (pulses / 2) * (60000 / sample_time_ms)
  float calculatedRpm = (count / 2.0) * (60000.0 / actualElapsed);
  
  // Constrain to max 2000 RPM
  rpm = constrain(calculatedRpm, 0, RPM_REPORT_MAX);
}

void registerSensor(char* value, char* unit_of_measurement) {
  char topic[128];
  char payload[512];
  
  // Build topic string
  char sensor_type[64];
  strcpy(
      sensor_type,
      strcmp(value, "pwm_connected") == 0
        ? "binary_sensor"
        : "sensor"
  );
  snprintf(topic, sizeof(topic), "homeassistant/%s/%s_%s/config", sensor_type, HASS_DEVICE_ID, value);
  
  // Build payload - common part
  int pos = snprintf(payload, sizeof(payload),
    "{"
    "\"name\":\"%s\","
    "\"state_topic\":\"%s\","
    "\"unique_id\":\"%s_%s\","
    "\"unit_of_measurement\":\"%s\","
    "\"value_template\":\"{{ value_json.%s }}\",",
    value, 
    mqtt_status_topic, 
    HASS_DEVICE_ID,
    value,
    unit_of_measurement,
    value);
  
  // Add conditional part
  if (strcmp(value, "temperature") == 0) {
    // The first (or any one) device must carry the info.
    pos += snprintf(payload + pos, sizeof(payload) - pos,
      "\"device_class\":\"%s\","
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

void registerNumber(char* value, const char *mqtt_control_topic, int minValue, int maxValue) {
  char topic[128];
  char payload[512];
  
  // Build topic string
  snprintf(topic, sizeof(topic), "homeassistant/number/%s_%s/config", HASS_DEVICE_ID, value);
  
  // Build payload - common part
  snprintf(payload, sizeof(payload),
    "{"
    "\"name\":\"%s control\","
    "\"command_topic\":\"%s\","
    "\"state_topic\":\"%s\","
    "\"value_template\":\"{{ value_json.%s }}\","
    "\"unique_id\":\"%s_%s_control\","
    "\"min\":%d,"
    "\"max\":%d,"
    "\"step\":1,"
    "\"device\":{"
    "\"identifiers\":[\"%s\"]"
    "}}", 
    value,
    mqtt_control_topic, 
    mqtt_status_topic,
    value,
    HASS_DEVICE_ID,
    value,
    minValue,
    maxValue,
    HASS_DEVICE_ID);
  
  if (client.publish(topic, payload, true)) {
    Serial.print("Reported self to ");
    Serial.println(topic);
    Serial.println(payload);
  } else {
    Serial.println("MQTT publish failed!");
  }
}

void registerSelect(const char* mqtt_control_topic) {
  char topic[128];
  char payload[512];
  
  // Build topic string
  snprintf(topic, sizeof(topic), "homeassistant/select/%s_select/config", HASS_DEVICE_ID);
  
  // Build payload - common part
  snprintf(payload, sizeof(payload),
    "{"
    "\"name\":\"PWM mode\","
    "\"command_topic\":\"%s\","
    "\"state_topic\":\"%s\","
    "\"value_template\":\"{{ value_json.pwm_mode }}\","
    "\"options\": [\"auto\", \"override\"],"
    "\"unique_id\":\"%s_pwm_mode\","
    "\"device\":{"
    "\"identifiers\":[\"%s\"]"
    "}}", 
    mqtt_control_topic, 
    mqtt_status_topic,
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

void reportStatus(float temp, int pwmValue, int rpmValue, float threshold, float maxTemp) {
  if (temp > 60.0 || temp < 0.0) {
    Serial.println("Not reporting a temperatue above 60.0 or below 0.0");
    return;
  }
  
  char payload[128];
  if (!FAN_ENABLED) {
    snprintf(
        payload,
        sizeof(payload),
        "{\"temperature\":%.1f}",
        temp
    );
  } else {
    int pwmPct = pwmValue * 100 / MAX_FAN_SPEED;
    char pwmMode[16];
    strcpy(pwmMode, overridePwm == -1.0 ? "auto" : "override");
    snprintf(
        payload,
        sizeof(payload),
        "{\"temperature\":%.2f,\"pwm\":%d,\"rpm\":%d,\"pwm_mode\":\"%s\",\"pwm_connected\":\"%s\",\"threshold\":%.1f,\"max_temp\":%.0f}",
        temp,
        pwmPct,
        rpmValue,
        pwmMode,
        relayEnabled > 0 ? "ON" : "OFF",
        threshold,
        maxTemp
    );
  }
  
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
    digitalWrite(LED_BUILTIN, LOW);
    wait_wifi();
  }
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (!client.connected() && millis() - lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = millis();
      mqtt_reconnect();
    }
    client.loop();
  }

  // Read temperature regardless of Wi-Fi connection
  sensors.requestTemperatures();
  float temp = -127.0;
  float ctemp = sensors.getTempCByIndex(0);
  if (ctemp > temp) {
    temp = ctemp;
  }

  if (FAN_ENABLED) {
    // Toggle with FAN_TEMP_HYSTERISIS, state lives in relayEnabled.
    int turnOnFan = 0;
    if (temp != 85.0) { // 85.0 is a bogus temperature on this specific device.
      if (relayEnabled && temp <= threshold - FAN_TEMP_HYSTERISIS) {
        turnOnFan = -1;
      } else if (!relayEnabled && temp >= threshold) {
        turnOnFan = 1;
      }
    }
    if (turnOnFan == 1) {
#ifdef RELAY_PIN
      digitalWrite(RELAY_PIN, HIGH);
#endif
      relayEnabled = true;
    } else if (turnOnFan == -1) {
#ifdef RELAY_PIN
      digitalWrite(RELAY_PIN, LOW);
#endif
      relayEnabled = false;
    }

    // Map temperature to PWM range (30°C = min speed, higher = faster)
    // Adjust the upper temperature limit as needed
    pwmValue = map(
        constrain(temp * 10, FAN_TEMP_MIN * 10, maxTemp * 10), 
        FAN_TEMP_MIN * 10, 
        maxTemp * 10,
        MIN_FAN_SPEED,
        MAX_FAN_SPEED
    );
    Serial.println("Auto-mapped pwm by temperature");

    if (overridePwm != -1.0) {
      Serial.println("Override set");
      pwmValue = overridePwm;
    } else if (!relayEnabled) {
      pwmValue = 0;
    }

    analogWrite(FAN_PWM, pwmValue);
    if (pwmValue > 0.0) {
      Serial.println("Fan is running");
      measureRPM(); // Runs for 1000ms
    } else {
      Serial.println("Fan is NOT running");
      rpm = 0;
      delay(1000);
    }
  } else {
    // No fan, just measure every minute.
    delay(60000);
  }

  if (WiFi.status() == WL_CONNECTED && millis() - lastReportTime > REPORT_INTERVAL_MS) {
    lastReportTime = millis();
    reportStatus(temp, pwmValue, rpm, threshold, maxTemp);
  }

  // Always print temperature and PWM for debugging
  Serial.print("(loop) Temp: ");
  if (FAN_ENABLED) {
    Serial.print(temp);
    Serial.print(" °C, PWM: ");
    Serial.print(pwmValue);

    Serial.print(", RPM: ");
    Serial.println(rpm);

  } else {
    Serial.println(" °C");
  }
  Serial.println("Sleeping lightly.");
  Serial.flush();
  // esp_light_sleep_start();
}
