#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// gps pins
#define RXPin 16
#define TXPin 17

// lora pins
#define ss 5
#define rst 14
#define dio0 2

// for approx altitude with BME280 (just incase GPS dosent work)
#define SEALEVELPRESSURE_HPA (1017.7)

struct BMEData {
  float temperature;
  float pressure;
  float humidity;
};

struct GPSData {
  double latitude;
  double longitude;
  double altitude;
  uint32_t date;
  uint32_t time;
};

Adafruit_BME280 bme; // I2C

TinyGPSPlus gps;

unsigned long delayTime;
uint16_t counter = 0;

void setup() {
  // initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("CanSAT");

  unsigned status;
  status = bme.begin(0x76);

  if (!status) {
    Serial.println("Could not find a valid BME280 sensor!");
  }

  Serial2.begin(9600, SERIAL_8N1, RXPin, TXPin);

  // setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  // replace the LoRa.begin(---E-) argument with your location's frequency
  // 915E6 for North America
  Serial.println("Initializing LoRa...");
  while (!LoRa.begin(915E6)) {
    delay(500);
  }

  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  Serial.print("Sending BME packet #");
  Serial.println(++counter);

  sendBMEData();

  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      Serial.print("Sending GPS packet #");
      Serial.println(++counter);
      sendGPSData();
      break;
    }
  }

  // no data received from the gps
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected");
  }

  delay(1000);
}

void sendBMEData() {
  // ~Alt
  // dont need to send this, can be caluated on the ground or later
  // float atmospheric = readPressure() / 100.0F;
  // return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));

  BMEData data = {
    bme.readTemperature(),
    bme.readPressure() / 100.0F,
    bme.readHumidity(),
  };

  Serial2.print(String(data.temperature));
  Serial2.print("C, "); // not using ° because it takes more than 1 byte
  Serial2.print(String(data.pressure));
  Serial2.print("hPa, ");
  Serial2.print(String(data.humidity));
  Serial2.println("%");

  uint8_t buffer[sizeof(char) + sizeof(uint16_t) + sizeof(BMEData)];
  buffer[0] = 'B';
  memcpy(buffer + sizeof(char), &counter, sizeof(uint16_t));
  memcpy(buffer + sizeof(char) + sizeof(uint16_t), &data, sizeof(BMEData));

  LoRa.beginPacket();
  LoRa.write(buffer, sizeof(char) + sizeof(uint16_t) + sizeof(BMEData));
  LoRa.endPacket();

}

void sendGPSData() {

  GPSData data = {
    gps.location.lat(),
    gps.location.lng(),
    gps.altitude.meters(),
    gps.time.value(),
    gps.date.value(),
  };
  
  Serial2.print("Lat: ");
  Serial2.print(data.latitude);
  Serial2.print("°, Lon: ");
  Serial2.print(data.longitude);
  Serial2.print("°, Alt: ");
  Serial2.print(data.altitude);
  Serial2.print("m, Date: ");
  Serial2.print(data.date % 100 + 2000); //  year
  Serial2.print("/");
  Serial2.print((data.date / 100) % 100); // month
  Serial2.print("/");
  Serial2.print(data.date / 10000); // day
  Serial2.print(", Time: ");
  Serial2.print(data.time / 1000000); // hour
  Serial2.print(":");
  Serial2.print((data.time / 10000) % 100); // minute
  Serial2.print(":");
  Serial2.println((data.time / 100) % 100); // second

  uint8_t buffer[sizeof(char) + sizeof(uint16_t) + sizeof(GPSData)];
  buffer[0] = 'G';
  memcpy(buffer + sizeof(char), &counter, sizeof(uint16_t));
  memcpy(buffer + sizeof(char) + sizeof(uint16_t), &data, sizeof(GPSData));

  LoRa.beginPacket();
  LoRa.write(buffer, sizeof(uint16_t) + sizeof(GPSData));
  LoRa.endPacket();
}
