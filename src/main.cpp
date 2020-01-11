/** Kingswood Monitor LoRa Environment Sensor
 * 
 *  sensor_environment_external-32U4RFM95LORA-arduino
 *
 *  Firmware for a LoRa based sensor. Detects which sensors are connected - access is 
 *  via "CompositeSensor" object which provides a unified interface for different sensor 
 *  types. CompositeSensor returns a data structure. Transmit this over LoRa.
 * 
 *  Two flashes: data received
 *  Four flashes: data not received
 * 
 */
#include <Arduino.h>
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <Ticker.h>

#include "sensor-utils.h"
#include "CompositeSensor.h"

#define DEVICE_ID "Greenhouse" // comment out if device has Sys.DeviceID()

// Radiohead addresses
#define SERVER_ADDRESS 1
// #define CLIENT_ADDRESS 2 // Outside
#define CLIENT_ADDRESS 3 // Greenhouse

#define FIRMWARE_VERSION "0.23"
#define SENSOR_TYPE "External Environment Sensor"
#define FIRMWARE_SLUG "sensor_environment_external-32U4RFM95LORA-arduino"
#define FIRMWARE_MCU "32U4RFM95LORA"
#define FIRMWARE_OS "arduino"

// number of seconds between transmissions
#define SLEEP_SECONDS 3

#define DEBUG true // set false to suppress debug info on Serial

// Radiohead pinsfor feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// board pin assignments
#define LED_BUILTIN 13 // red onboard LED

// Initialise sensors
CompositeSensor mySensor;

// Embedis
const size_t EEPROM_SIZE = E2END + 1;
// Embedis embedis(Serial);

// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// Timers for LED flashing
void flashLED(int times, int millis);

void setup()
{
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(2000);

  utils::printBanner(SENSOR_TYPE, FIRMWARE_SLUG, FIRMWARE_VERSION, DEVICE_ID);

  // start LoRa
  if (manager.init())
  {
    Serial.print("LoRa client started, ID:");
    Serial.println(CLIENT_ADDRESS);
  }

  mySensor.begin();
  // readSensors.start();
}

void loop()
{
  // code resumes here on wake.

  CompositeSensor::SensorReadings readings = mySensor.readSensors();

  // serialise the JSON for transmission
  uint8_t serialData[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

  byte zize = sizeof(readings);
  memcpy(buf, &readings, zize);

  // send message to manager server
  if (manager.sendtoWait(buf, zize, SERVER_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
    {
      Serial.print("Server:");
      Serial.println((char *)buf);
      flashLED(CLIENT_ADDRESS, 150);
    }
    else
    {
      Serial.println("No reply, is rf95_reliable_datagram_server running?");
      flashLED(1, 1000);
    }
  }
  else
    Serial.println("sendtoWait failed");

  delay(SLEEP_SECONDS * 1000);
}

void flashLED(int times, int millis)
{
  for (int i = 0; i < times; ++i)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(millis);
    digitalWrite(LED_BUILTIN, LOW);
    delay(millis);
  }
}