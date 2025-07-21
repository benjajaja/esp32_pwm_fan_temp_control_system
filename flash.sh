#!/usr/bin/env bash
set -euo pipefail

DEVICE="$1"
ENV_FILE=".env.$DEVICE"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Missing $ENV_FILE"
  exit 1
fi

set -a
source "$ENV_FILE"
set +a

FLAGS=(
  -DWIFI_SSID=\"${WIFI_SSID}\"
  -DWIFI_PASSWORD=\"${WIFI_PASSWORD}\"
  -DMQTT_SERVER=\"${MQTT_SERVER}\"
  -DMQTT_PORT=${MQTT_PORT}
  -DHASS_DEVICE_ID=\"${HASS_DEVICE_ID}\"
  -DHASS_DEVICE_NAME=\"${HASS_DEVICE_NAME}\"
)

# Join the flags array into one space-separated string:
FLAGS_STR="${FLAGS[*]}"

sudo arduino-cli compile \
  --fqbn esp32:esp32:esp32doit-devkit-v1 \
  -v \
  --build-property "build.extra_flags=${FLAGS_STR}" \
  .

sudo arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32doit-devkit-v1 .
sudo arduino-cli monitor -p /dev/ttyUSB0 -c 115200

