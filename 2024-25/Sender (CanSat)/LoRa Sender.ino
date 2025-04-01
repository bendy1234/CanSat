#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ESP32Servo.h>

// LoRa
#define csPin 10      // LoRa radio chip select
#define resetPin 40
#define irqPin 41

#define servoPin 36

Adafruit_BME680 bme;  // Address is changed to 0x76 in `setup()`
Servo myservo;

byte msgCount = 0;
long lastSendTime = 0;
int interval = 2000;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Duplex with BME680");

  // Initialize servo
  myservo.attach(servoPin);
  myservo.write(90);  // set initial position to 90 degrees

  // Initialize BME680 sensor
  Wire.begin(8, 9);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (true);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa init succeeded.");
}

void loop() {
  // check for incomeing packets
  onReceive(LoRa.parsePacket());

  if (millis() - lastSendTime > interval) {
    // Read sensor data
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
      Serial.println("Failed to begin reading :(");
      return;
    }

    if (!bme.endReading()) {
      Serial.println("Failed to complete reading :(");
      return;
    }

    // Format sensor data into a single message
    String msg = String(millis() / 1000.0, 1) + "," + 
                  String(bme.temperature, 1) + "," + 
                  String(bme.pressure / 100.0, 1) + "," + 
                  String(bme.humidity, 1) + "," + 
                  String(bme.gas_resistance / 1000.0, 1);
    
    sendMessage(msg);
    lastSendTime = millis();            // timestamp the message
    LoRa.receive();
  }
}

void sendMessage(String msg) {
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  msgCount++;
  Serial.println("SENT: " + msg);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  // get message
  String msg = LoRa.readString();

  Serial.print("RECV: ");
  Serial.println(msg);

  // Check for servo command
  if (msg.startsWith("SERVO:")) {
    int angle = msg.substring(6).toInt();  // Get num after "SERVO:"
    
    // Validate angle
    if (angle >= 0 && angle <= 180) {
      myservo.write(angle);
      Serial.println("Servo set to: " + String(angle));
      delay(10);
      sendMessage("SERVO: " + String(angle));
    } else {
      Serial.println("Invalid servo angle");
      sendMessage("SERVO: Invalid angle");
    }

  }
}
