
/**
 * Configuration file for a LoRa Environment Sensor
 * 
 */

#define MQTT_TOPIC_ROOT "greenhouse"

// firmware info
#define FIRMWARE_NAME "Environment Sensor (Greenhouse)"
#define FIRMWARE_VERSION "0.2"
#define FIRMWARE_SLUG "sensor-environment-greenhouse-32U4RFM95LORA-arduino"
#define FIRMWARE_MCU "32U4RFM95LORA"
#define FIRMWARE_OS "arduino"
#define DEVICE_ID "002" // comment out if device has Sys.DeviceID()
#define JSON_PROTOCOL "1.1"

// battery info
#define BATTERY_ACTIVE true

// set true to sleep between transmissions to conserve battery
#define SLEEP_MODE false
// number of seconds between transmissions
#define SLEEP_SECONDS 3