#include <SPI.h>
#include <LoRa.h>
#include <RS-FEC.h> // https://github.com/simonyipeter/Arduino-FEC/tree/5d2164e6731d9f96e01aaf94d314d26e242f98e5

// LoRa
#define ss 10
#define rst 2
#define dio0 1

RS::ReedSolomon<12, 6> shortRS;
RS::ReedSolomon<40, 8> longRS;

void setup() {
  // initialize Serial Monitor
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Receiver");

  // setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  // 915E6 for North America
  while (!LoRa.begin(907E6)) {
    Serial.print(".");
    delay(500);
  }

  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  // try to parse packet
  if (LoRa.parsePacket()) {
    Serial.print("Received packet with RSSI ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" ");

    // read packet
    uint8_t data[48];
    uint8_t size = LoRa.readBytes(data, sizeof(data));
    parseData(data, size);
  
    // only send packet after receiving one
    if (Serial.available() > 0) {
      delay(10);
      String command = Serial.readStringUntil('\n');

      Serial.print("Sending new command '");
      Serial.print(command);
      Serial.println("'");

      LoRa.beginPacket();
      uint8_t data[18];
      shortRS.Encode(command.c_str(), data);
      LoRa.write(data, sizeof(data));
      LoRa.endPacket();

      LoRa.receive();
    }
  }
}

void parseData(uint8_t* data, uint8_t size) {
  if (size > 18) {
    float sensorData[10];
    longRS.Decode(data, sensorData);

    Serial.print(sensorData[0]);
    Serial.print("s, ");
    Serial.print(sensorData[1]);
    Serial.print("∘c, ");
    Serial.print(sensorData[2]);
    Serial.print("Hpa, ");
    Serial.print(sensorData[3]);
    Serial.print("%, est pos: ");
    Serial.print(sensorData[4]);
    Serial.print(", ");
    Serial.print(sensorData[5]);
    Serial.print(", ");
    Serial.print(sensorData[6]);
    Serial.print(", gps: ");
    Serial.print(sensorData[7]);
    Serial.print(", ");
    Serial.print(sensorData[8]);
    Serial.print(", altitude: ");
    Serial.println(sensorData[9]);
  } else {
    char msg[13];
    msg[12] = 0;
    shortRS.Decode(data, msg);
    Serial.println(String(msg)); // am lazy
  }
}

// TODO
void sendCommand(String command) {

}
