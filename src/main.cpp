/* Kingswood Monitor Environment Sensor (LoRa)
 *
 * Firmware for a LoRa based sensor
 * Reads sensor data and battery voltage
 * Package as JSON strind
 * Transmits
 *
 * NOTE:
 * Implements 'sleep' function between data transmission, which disables the USB serial line.
 * Reset the device before flashing.
 *
 *  Format
 *  ------------------------------------------------------------
    packetID = 1234
    protocol n = 1.1

    device
      ID = ESP8266-001
      type = ESP8266
      firmware
        version n = 0.2
        slug = sensor-environment-outside-32U4RFM95LORA-arduino
        os = mongoose
      battery
        active b = true
        voltage n = 3.8
      lora
        RSSI n = -98
        SNR n = 23
        frequencyError n = 12234

    measurement
      temperature n = 21.1
      humidity n = 87
      co2 = 567
      lux n = 600
      mbars n = 1023

    status
      message = OK
      description = All's well
    ---------------------------------------------------------------
 *
 */

#include <Wire.h>
#include <LoRa.h>
#include <Ticker.h> // https://github.com/sstaub/Ticker.git
#include <Adafruit_SleepyDog.h>
#include <ArduinoJson.h>

#include "SparkFun_SCD30_Arduino_Library.h"
#include "Adafruit_VEML7700.h"
#include "DFRobot_BMP388_I2C.h"
#include "sensor-utils.h"

#define DEBUG true // set false to suppress debug info on Serial

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

// board pin assignments
#define LED_BUILTIN 13 // red onboard LED
#define VBATPIN A9     // for measuring battery voltage

// SCD30 pin assignments
#define SHT15dataPin A4
#define SHT15clockPin A5

// feather32u4 LoRa pin assignment
#define NSS 8    // New Slave Select pin
#define NRESET 4 // Reset pin
#define DIO0 7   // DIO0 pin

// set true to sleep between transmissions to conserve battery
#define SLEEP_MODE false
// number of seconds between transmissions
#define SLEEP_SECONDS 3

const size_t capacity = 2 * JSON_OBJECT_SIZE(2) + 2 * JSON_OBJECT_SIZE(3) + 3 * JSON_OBJECT_SIZE(5);
StaticJsonDocument<capacity> doc;

JsonObject device = doc.createNestedObject("device");
JsonObject device_firmware = device.createNestedObject("firmware");
JsonObject device_battery = device.createNestedObject("battery");
JsonObject device_lora = device.createNestedObject("lora");
JsonObject measurement = doc.createNestedObject("measurement");
JsonObject status = doc.createNestedObject("status");

// Initialise sensors
SCD30 airSensor;
Adafruit_VEML7700 veml = Adafruit_VEML7700();
DFRobot_BMP388_I2C bmp388;

unsigned int packetID;

void setup()
{
  Serial.begin(115200);
  delay(2000);

  utils::printBanner(FIRMWARE_NAME, FIRMWARE_VERSION, FIRMWARE_SLUG, JSON_PROTOCOL, FIRMWARE_MCU, FIRMWARE_OS, DEVICE_ID);

  pinMode(LED_BUILTIN, OUTPUT);
  LoRa.setPins(NSS, NRESET, DIO0);
  digitalWrite(LED_BUILTIN, LOW);

  airSensor.begin();
  if (!airSensor.dataAvailable())
  {
    Serial.println("SCD30 air sensor not found");
    while (1)
      ;
  }
  Serial.println("SCD30 air sensor started");
  airSensor.setMeasurementInterval(4);
  airSensor.setAmbientPressure(1013); // move this into loop and use barometric presssure

  if (!veml.begin())
  {
    Serial.println("VEML7700 light sensor not found");
    while (1)
      ;
  }
  Serial.println("VEML7700 light sensor started");

  veml.setGain(VEML7700_GAIN_1);
  veml.setIntegrationTime(VEML7700_IT_800MS);
  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);

  while (bmp388.begin())
  {
    Serial.println("BMP388 pressure sensor not found");
    delay(1000);
  }
  Serial.println("BMP388 pressure sensor started");
  Serial.println();

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
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  device_battery["active"] = BATTERY_ACTIVE;
  device_battery["voltage"] = 3.8;

  // lora - NULL for sending device
  device_lora["RSSI"] = nullptr;
  device_lora["SNR"] = nullptr;
  device_lora["frequencyError"] = nullptr;

  // sensors
  measurement["temperature"] = airSensor.getTemperature();
  measurement["humidity"] = airSensor.getHumidity();
  measurement["co2"] = airSensor.getCO2();
  measurement["lux"] = veml.readLux();
  measurement["mbars"] = bmp388.readPressure() / 100.0;

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