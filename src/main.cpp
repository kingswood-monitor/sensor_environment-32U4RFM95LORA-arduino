/** Kingswood Monitor LoRa Environment Sensor
 * 
 *  sensor-environment-32U4RFM95LORA-arduino
 *
 *  Firmware for a LoRa based sensor. Detects which sensors are connected - access is 
 *  via "CompositeSensor" object which provides a unified interface for different sensor 
 *  types. Packages readings in a JSON string, serialises it, and transmits it over LoRa.
 * 
 *  Set configuration variables in 'config.h'. 
 * 
 *  NOTE: Implements 'sleep' function between data transmission, which disables 
 *  the USB serial line. Reset the device before flashing.
 * 
 */
#include <LoRa.h>
#include <Ticker.h>
#include <Adafruit_SleepyDog.h>
#include <ArduinoJson.h>

#include "sensor-utils.h"
#include "CompositeSensor.h"
#include "config.h"

#define DEBUG true // set false to suppress debug info on Serial

// board pin assignments
#define LED_BUILTIN 13 // red onboard LED

// feather32u4 LoRa pin assignment
#define NSS 8    // New Slave Select pin
#define NRESET 4 // Reset pin
#define DIO0 7   // DIO0 pin

const size_t capacity = 2 * JSON_OBJECT_SIZE(2) + 2 * JSON_OBJECT_SIZE(3) + 3 * JSON_OBJECT_SIZE(5);
StaticJsonDocument<capacity> doc;

JsonObject device = doc.createNestedObject("device");
JsonObject device_firmware = device.createNestedObject("firmware");
JsonObject device_battery = device.createNestedObject("battery");
JsonObject device_lora = device.createNestedObject("lora");
JsonObject measurement = doc.createNestedObject("measurement");
JsonObject status = doc.createNestedObject("status");

// Initialise sensors
CompositeSensor mySensor;

unsigned int packetID;

void setup()
{
  Serial.begin(115200);
  delay(2000);

  LoRa.setPins(NSS, NRESET, DIO0);
  digitalWrite(LED_BUILTIN, LOW);

  utils::printBanner(FIRMWARE_NAME, FIRMWARE_VERSION, FIRMWARE_SLUG, JSON_PROTOCOL, FIRMWARE_MCU, FIRMWARE_OS, DEVICE_ID);

  mySensor.begin();
  if (mySensor.hasSCD30)
    Serial.println("SCD30 started");
  if (mySensor.hasBMP388)
    Serial.println("BMP388 started");
  if (mySensor.hasVEML7700)
    Serial.println("VEML7700 started");
  if (mySensor.hasHDC1080)
    Serial.println("HDC1080 started");

  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed.");
    while (1)
      ;
  }
  Serial.println("LoRa started");
  packetID = 0;
}

void loop()
{
  // code resumes here on wake.

  // serialise the JSON for transmission
  char serialData[255];

  ++packetID;
  doc["packetID"] = packetID;
  doc["protocol"] = JSON_PROTOCOL;

  // device
  device["id"] = DEVICE_ID;
  device["type"] = FIRMWARE_MCU;

  // firmware
  device_firmware["version"] = FIRMWARE_VERSION;
  device_firmware["slug"] = FIRMWARE_SLUG;
  device_firmware["os"] = FIRMWARE_OS;

  // battery
  device_battery["active"] = BATTERY_ACTIVE;
  device_battery["voltage"] = mySensor.readBattery();

  // lora - NULL for sending device
  device_lora["RSSI"] = nullptr;
  device_lora["SNR"] = nullptr;
  device_lora["frequencyError"] = nullptr;

  // sensors
  measurement["temperature"] = mySensor.readTemperature();
  measurement["humidity"] = mySensor.readHumidity();
  measurement["co2"] = mySensor.readCO2();
  measurement["lux"] = mySensor.readLight();
  measurement["mbars"] = mySensor.readPressure();

  // status
  status["message"] = "OK";
  status["description"] = nullptr;

  serializeJson(doc, serialData);

  // send in async / non-blocking mode
  while (LoRa.beginPacket() == 0)
  {
    delay(100);
  }
  LoRa.beginPacket();
  LoRa.print(serialData);
  LoRa.endPacket(true); // true = async / non-blocking mode

  // signal transmission
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);

  // log to serial port
  Serial.println("TX Packet: ");
  if (DEBUG)
  {
    Serial.println("---");
    serializeJsonPretty(doc, Serial);
    Serial.println();
  }

  // sleep
  if (SLEEP_MODE)
  {
    Watchdog.sleep(SLEEP_SECONDS * 1000);
  }
  else
  {
    delay(SLEEP_SECONDS * 1000);
  }
}