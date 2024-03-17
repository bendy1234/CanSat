#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// VV lora pins VV
#define ss 5
#define rst 14
#define dio0 2
// VV BMP 280 pins VV
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
// VV for approx altitude with BMP280 (just incase GPS dosent work) VV
#define SEALEVELPRESSURE_HPA (1017.7)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

int counter = 0;

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender/BMP 280 test");

    unsigned status;

    status = bme.begin(0x76);

    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
    }
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  Serial.println("Sending packet #");
  Serial.print(counter);

  sendValues();

  counter++;

  delay(1000);
}
void sendValues() {
    LoRa.beginPacket();
    LoRa.print("Temp= ");
    LoRa.print(bme.readTemperature());
    LoRa.print(" C");

    LoRa.println("Pressr= ");
    LoRa.print(bme.readPressure() / 100.0F);
    LoRa.print(" hPa");

    LoRa.println("~Alt= ");
    LoRa.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    LoRa.print(" m");

    LoRa.println(bme.readHumidity());
    LoRa.print("% Humid");

    LoRa.println("Packet #");
    LoRa.print(counter);

    LoRa.println();
    LoRa.endPacket();
    
}