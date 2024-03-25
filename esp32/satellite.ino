#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include <TinyGPS.h>
// #include <HardwareSerial.h>

// lora pins
#define ss 5
#define rst 14
#define dio0 2

// BMP 280 pins
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// for approx altitude with BMP280 (just incase GPS dosent work)
#define SEALEVELPRESSURE_HPA (1017.7)

struct BMEData
{
  float temperature;
  float pressure;
  float humidity;
};

// struct GPSData
// {
//   float latitude;
//   float longitude;
//   int altitude;
//   int date;
//   int time;
// };

Adafruit_BME280 bme; // I2C
// Adafruit_BME280 bme(BME_CS); // hardware SPI
// Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// TinyGPS gps;
// HardwareSerial SerialGPS(0, 1);

unsigned long delayTime;
uint16_t counter = 0;

void setup()
{
  // initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender/BMP 280 test");

  unsigned status;
  status = bme.begin(0x76);

  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor!");
  }
  // setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  // replace the LoRa.begin(---E-) argument with your location's frequency
  // 915E6 for North America
  while (!LoRa.begin(915E6))
  {
    Serial.println(".");
    delay(500);
  }

  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop()
{
  Serial.println("Sending packet #");
  Serial.print(counter);
  // TODO: decide whether to send the data in one big packet or not
  sendBMEData();
  counter++;

  // Serial.println("Sending packet #");
  // Serial.print(counter);
  // sendGPSData();
  // counter++;

  delay(1000);
}

void sendBMEData()
{
  // ~Alt
  // dont need to send this, can be caluated on the ground or later
  // float atmospheric = readPressure() / 100.0F;
  // return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));

  BMEData data = {
    bme.readTemperature(),
    bme.readPressure(),
    bme.readHumidity(),
  };

  uint8_t buffer[sizeof(uint16_t) + sizeof(BMEData)];
  memcpy(buffer + 1, &counter, sizeof(uint16_t));
  memcpy(buffer + 1 + sizeof(uint16_t) , &data, sizeof(BMEData));

  LoRa.beginPacket();
  LoRa.write('B');
  LoRa.write(buffer, sizeof(uint16_t) + sizeof(BMEData));
  LoRa.endPacket();
}

// void sendGPSData()
// {
//   GPSData data = {
//     0.0,
//     0.0,
//     0,
//     0,
//     0
//   };

//   uint8_t buffer[sizeof(uint16_t) + sizeof(GPSData)];
//   memcpy(buffer, &counter, sizeof(uint16_t));
//   memcpy(buffer + sizeof(uint16_t), &data, sizeof(GPSData));

//   // send the gps data
//   LoRa.beginPacket();
//   LoRa.write('G');
//   LoRa.write(buffer, sizeof(uint16_t) + sizeof(GPSData));

//   LoRa.endPacket();

// }
