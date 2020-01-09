
/**
 * Configuration file for a LoRa Environment Sensor
 * 
 */
// firmware info
#define FIRMWARE_NAME "Environment Sensor"
#define FIRMWARE_VERSION "0.2"
#define FIRMWARE_SLUG "sensor-environment-32U4RFM95LORA-arduino"
#define FIRMWARE_MCU "32U4RFM95LORA"
#define FIRMWARE_OS "arduino"
#define DEVICE_ID "32U4RFM95LORA-002" // comment out if device has Sys.DeviceID()

// battery info
#define BATTERY_ACTIVE true

// set true to sleep between transmissions to conserve battery
#define SLEEP_MODE false
// number of seconds between transmissions
#define SLEEP_SECONDS 3