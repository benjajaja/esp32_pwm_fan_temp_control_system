# ESP32 Fan Control System for PC, Server, and Rack Temperature Monitoring

## Description
This project is an open-source system designed to control multiple 12V PWM fans based on temperature readings from DS18B20 sensors. It uses an ESP32 microcontroller and supports MQTT over Wi-Fi to report sensor data and fan speeds. The system ensures continuous operation even when Wi-Fi or MQTT connections fail.

## Features
- **Fan Control:** Adjusts fan speed based on temperature ranges.
- **Temperature Monitoring:** Supports multiple DS18B20 sensors on a single pin.
- **MQTT Integration:** Reports fan speed and temperature data over MQTT.
- **Fail-Safe Operation:** Maintains functionality if Wi-Fi or MQTT connectivity is lost.
- **Configurable Parameters:** User-configurable temperature thresholds, PWM frequency, and more.

## License
This project is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)**.

## Hardware Requirements
1. **ESP32 Microcontroller**
2. **DS18B20 Temperature Sensors** (1 or more)
3. **12V PWM Fans**
4. **12V Power Supply**
5. Resistors and jumper wires for connections

## Software Requirements
- **Arduino IDE**
- ESP32 Board Manager (Version 3.0.7)
- Arduino Libraries:
  - `WiFi`
  - `PubSubClient`
  - `OneWire`
  - `DallasTemperature`

## Installation and Setup
1. Clone or download this repository.
2. Open the provided `.ino` file in the Arduino IDE.
3. Install the required libraries from the Arduino Library Manager.
4. Configure the following user variables in the `USER CONFIGURATION SECTION`:
   - Wi-Fi credentials (`ssid` and `password`)
   - MQTT broker information (`mqtt_server`, `mqtt_port`, etc.)
   - Number of sensors and fans (`NUM_SENSORS`, `NUM_FANS`)
   - Temperature thresholds (`MIN_TEMP`, `MAX_TEMP`)
   - PWM frequency (`PWM_FREQUENCY`)
5. Connect the hardware as per the pin definitions in the code.
6. Upload the code to your ESP32.

## How to Use
- The system will automatically adjust fan speed based on temperature readings.
- Monitor the fan speeds and temperatures via the MQTT broker.
- If the Wi-Fi connection fails, the system will continue to adjust fans based on sensor readings.

## Customization
- **Additional Fans and Sensors:** Update `NUM_SENSORS` and `NUM_FANS` variables and configure the corresponding GPIO pins.
- **PWM Frequency:** Modify the `PWM_FREQUENCY` variable to match your fan requirements.
- **Temperature Thresholds:** Adjust `MIN_TEMP` and `MAX_TEMP` to suit your cooling needs.

## Troubleshooting
1. Ensure the DS18B20 sensors are connected with an appropriate pull-up resistor.
2. Verify the MQTT broker details.
3. Check serial monitor output for debugging information.

## Author
Claude Wolter

---

## Contributions
Feel free to contribute to this project by submitting issues or pull requests.

## Disclaimer
This project is for non-commercial use only. Please adhere to the licensing terms.
