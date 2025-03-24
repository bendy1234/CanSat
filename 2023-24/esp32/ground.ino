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
  double latitude;
  double longitude;
  double altitude;
  uint32_t date;
  uint32_t time;
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
    uint8_t buffer[sizeof(char) + sizeof(uint16_t) + sizeof(GPSData)] = { 0 };

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
    if ((bytesRead - sizeof(char) - sizeof(uint16_t)) == sizeof(BMEData)) {
      Serial.println("' Expecting BME data");
      type = 'B';
    }
    else if ((bytesRead - 3) == sizeof(GPSData)) {
      Serial.println("' Expecting GPS data");
      type = 'G';
    }
    else {
      Serial.println("' Skipping!");
      return;
    }
  }

  uint16_t packetNumber;
  memcpy(&packetNumber, buffer + sizeof(char), sizeof(uint16_t));
  Serial.print("Packet #");
  Serial.print(packetNumber);
  Serial.print(" ");

  // no need to worry about memory isuses
  if (type == 'B') {
    BMEData data;
    memcpy(&data, buffer + sizeof(char) + sizeof(uint16_t), sizeof(BMEData));
    Serial.print(data.temperature);
    Serial.print("°C, ");
    Serial.print(data.pressure);
    Serial.print("hPa, ");
    Serial.print(data.humidity);
    Serial.println("%");
  }
  else if (type == 'G') {
    GPSData data;
    memcpy(&data, buffer + sizeof(char) + sizeof(uint16_t), sizeof(GPSData));

    Serial.print("Lat: ");
    Serial.print(data.latitude, 6);
    Serial.print("°, Lon: ");
    Serial.print(data.longitude, 6);
    Serial.print("°, Alt: ");
    Serial.print(data.altitude);
    Serial.print("m, Date: ");

    Serial.print(data.date % 100 + 2000); //  year
    Serial.print("/");
    Serial.print((data.date / 100) % 100); // month
    Serial.print("/");
    Serial.print(data.date / 10000); // day

    Serial.print(", Time: ");
    Serial.print(data.time / 1000000); // hour
    Serial.print(":");
    Serial.print((data.time / 10000) % 100); // minute
    Serial.print(":");
    Serial.println((data.time / 100) % 100); // second
  }
}
