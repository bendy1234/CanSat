#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>

// LoRa
#define csPin 10      // LoRa radio chip select
#define resetPin 40
#define irqPin 41

#define aileronServoPin 38
// #define servoPin2 39
#define rudderServoPin 42
// #define servoPin4 45

Adafruit_BME280 bme;  // Address is changed to 0x76 in `setup()`
Servo aileronServo;
// Servo servo2;
Servo rudderServo;
// Servo servo4;

byte msgCount = 0;
long lastSendTime = 0;
int interval = 2000;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Duplex with BME280");

  // Initialize servos
  aileronServo.attach(aileronServoPin);
  rudderServo.attach(rudderServoPin);

  // set initial position to 90 degrees
  aileronServo.write(90);
  rudderServo.write(90);

  // Initialize BME680 sensor
  Wire.begin(8, 9);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
    while (true);
  }

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
    
    // Format sensor data into a single message
    String msg = String(millis() / 1000.0, 1) + "," + 
                  String(bme.readTemperature(), 1) + "," + 
                  String(bme.readPressure() / 100.0, 1) + "," + 
                  String(bme.readHumidity(), 1);
    
    sendMessage(msg);
    lastSendTime = millis();            // timestamp the message
    LoRa.receive();
  }
}

void servoCommand(String command) {
  int angle = command.substring(3).toInt();
  // Validate angle
  if (angle < 0 || angle > 180) {
    Serial.println("Invalid servo angle");
    sendMessage("SERVO: Invalid angle");
    return;
  }

  if (command.startsWith("ail")) {
    aileronServo.write(angle);
  } else if (command.startsWith("rud")) {
    rudderServo.write(angle);
  } else {
    Serial.print("Unknown servo: ");
    Serial.println(command.substring(0, 2));

    delay(10);
    sendMessage("SERVO: Unknown");
    return;
  }

  Serial.print("Servo ");
  Serial.print(command.substring(0, 2));
  Serial.println("set to: " + String(angle));

  delay(10);
  sendMessage("SERVO: " + command);
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
    servoCommand(msg.substring(6));
  }
}
