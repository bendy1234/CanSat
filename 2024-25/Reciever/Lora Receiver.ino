#include <SPI.h>
#include <LoRa.h>

// LoRa
#define ss 10
#define rst 2
#define dio0 1

void setup() {
  // initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  // setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  // 915E6 for North America
  while (!LoRa.begin(915E6)) {
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
    Serial.print(" '");

    // read packet
    String LoRaData = LoRa.readString();

    Serial.print(LoRaData);
    Serial.println("'");
  
    // only send packet after receiving one
    if (Serial.available() > 0) {
      delay(10);
      int new_angle = Serial.readStringUntil('\n').toInt();

      Serial.print("Sending new angle ");
      Serial.println(new_angle);

      LoRa.beginPacket();
      LoRa.print("SERVO:");
      LoRa.print(new_angle);
      LoRa.endPacket();

      LoRa.receive();
    }
  }
}
