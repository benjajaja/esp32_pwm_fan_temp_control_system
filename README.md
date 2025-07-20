# ESP32 Fan Control System for PC, Server, and Rack Temperature Monitoring

## Description
This project is an open-source system designed to control multiple 12V PWM fans based on temperature readings from DS18B20 sensors. It uses an ESP32 microcontroller and supports MQTT over Wi-Fi to report sensor data and fan speeds. The system ensures continuous operation even when Wi-Fi or MQTT connections fail.

## Features
- **Fan Control:** Adjusts fan speed based on temperature ranges.
- **Temperature Monitoring:** Supports multiple DS18B20 sensors on a single pin.
- **MQTT Integration:** Reports fan speed and temperature data over MQTT.
- **Fail-Safe Operation:** Maintains functionality if Wi-Fi or MQTT connectivity is lost.
- **Configurable Parameters:** User-configurable temperature thresholds, PWM frequency, and more.

## ESP32 only
>[!IMPORTANT]
>This sketch is not compatible with ESP8266 due to the use of the ESP32-specific LEDC API for PWM control. The ESP8266 is not suitable for fans requiring high PWM frequencies above 1kHz, as it lacks support for higher PWM rates.

## License
This project is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)**.

## Hardware Requirements
1. **ESP32 Microcontroller**
2. **DS18B20 Temperature Sensors** (1 or more)
  - A ~4.7kΩ resitor may be necessary
3. **5V PWM Fans** (Noctua 5V models work fine with usb power `VIN` pin)

## Software Requirements
- **Arduino IDE**
- ESP32 Board Manager (Version 3.0.7)
- Arduino Libraries:
  - `WiFi`
  - `PubSubClient`
  - `OneWire`
  - `DallasTemperature`
- Python 3

## Installation and Setup
1. Use `arduino-cli` to install required libraries.
2. Create an `.env.mydevice` file and fill in the sensitive constants:
  - `WIFI_SSID`
  - `WIFI_PASSWORD`
  - `MQTT_SERVER`
  - `MQTT_PORT`
  - `HASS_DEVICE_ID`
  - `HASS_DEVICE_NAME`
3. Connect the hardware as per the pin definitions in the code, edit the code to match if necessary.
4. Run `./flash.sh mydevice`
  - Or edit the file if you don't need/want to attach to serial ttyUSB immediately.

## How to Use
- The system will automatically adjust fan speed based on temperature readings.
- The device reports itself to home-assistant MQTT auto-discovery.
- Monitor the fan speeds and temperatures via the MQTT broker.
- If the Wi-Fi connection fails, the system will continue to adjust fans based on sensor readings.

## Customization
- **Additional Fans and Sensors:** Update `NUM_SENSORS` and `NUM_FANS` variables and configure the corresponding GPIO pins.
- **PWM Frequency:** Modify the `PWM_FREQUENCY` variable to match your fan requirements.
- **Temperature Thresholds:** Adjust `MIN_TEMP` and `MAX_TEMP` to suit your cooling needs.

## Troubleshooting
1. If you get temperature readings of -127°C, ensure the DS18B20 sensors are connected with an appropriate pull-up resistor. The build-in resistor might not be sufficient, add a ~4.7kΩ resistor to pull up the the pin to 3V3. Connect it between the 3.3 and the data pin you picked.
2. Verify the MQTT broker details.
3. Check serial monitor output for debugging information.
4. Home-assistant should automatically discover the device.
5. Some GPIO pins can interfere with the automatic boot/flash mode of the upload, first try removing any connections, and if that works, pick another pin.

## Author
Claude Wolter, Benjamin Große

---

## Contributions
Feel free to contribute to this project by submitting issues or pull requests.

## Disclaimer
This project is for non-commercial use only. Please adhere to the licensing terms.
