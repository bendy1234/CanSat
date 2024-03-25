#include <LoRa.h>

// lora pins
#define ss 5
#define rst 14
#define dio0 2

struct BMEData {
  float temperature;
  float pressure;
  float humidity;
};

struct GPSData {
  float latitude;
  float longitude;
  long altitude;
  int date; // ddmmyy
  int time; // hhmmsscc
};

void setup() {
  // initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);


  // setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  // 915E6 for North America
  while (!LoRa.begin(915E6)) {
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
  if (LoRa.parsePacket()) {
    // use sizeof(GPSData) for now because it's larger
    uint8_t buffer[sizeof(char) + sizeof(uint16_t) + sizeof(GPSData)] = {0};

    int bytesRead = 0;
    while (LoRa.available()) {
      buffer[bytesRead] = LoRa.read();
      bytesRead++;
    }
    // okok got all of the data
    parseData(buffer, bytesRead);
  }
}

void parseData(uint8_t* buffer, int bytesRead) {
  char type = buffer[0];
  if (type != 'B' && type != 'G') {
    Serial.print("Unknown packet type: '");
    Serial.print(type);
    if ((bytesRead - 3) == sizeof(BMEData)) {
      Serial.println("' Expecting BME data");
      type = 'B';
    } else if ((bytesRead - 3) == sizeof(GPSData)) {
      Serial.println("' Expecting GPS data");
      type = 'G';
    } else {
      Serial.println("' Skipping!");
      return;
    }
  }

  uint16_t packetNumber;
  memcpy(&packetNumber, buffer + 1, sizeof(uint16_t));
  Serial.print("Packet # ");
  Serial.print(packetNumber);
  Serial.print(" ");

  // no need to worry about memory isuses
  if (type == 'B') {
    BMEData data;
    memcpy(&data, buffer + 3, sizeof(BMEData));
    Serial.print(data.temperature);
    Serial.print("°C, ");
    Serial.print(data.pressure);
    Serial.print("hPa, ");
    Serial.print(data.humidity);
    Serial.println("%");
  } else if (type == 'G') {
    GPSData data;
    memcpy(&data, buffer + 3, sizeof(GPSData));
    Serial.print("Lat: ");
    Serial.print(data.latitude);
    Serial.print("°, Lon: ");
    Serial.print(data.longitude);
    Serial.print("°, Alt: ");
    Serial.print(data.altitude / 100.0);
    Serial.print("m, Date: ");
    Serial.print(data.date); // ddmmyy
    Serial.print(", Time: ");
    Serial.println(data.time); // hhmmsscc
  }
}
