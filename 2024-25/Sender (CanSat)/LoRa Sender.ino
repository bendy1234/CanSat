/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ESP32Servo.h>
// Create BME680 object with I2C address 0x76
Adafruit_BME680 bme; // Default is 0x77, we'll set 0x76 in begin()
Servo myservo;  // create servo object

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 40;       // LoRa radio reset
const int irqPin = 41;         // change for your board; must be a hardware interrupt pin
const int servoPin = 36;       // servo pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex with BME680");

  // Initialize servo
  myservo.attach(servoPin);
  myservo.write(90);  // set initial position to 90 degrees

  // Initialize BME680 sensor
  Wire.begin(8, 9);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
  if (millis() - lastSendTime > interval) {
    // Read sensor data
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
      Serial.println("Failed to begin reading :(");
      return;
    }
    delay(200); // Wait for reading to complete

    if (!bme.endReading()) {
      Serial.println("Failed to complete reading :(");
      return;
    }

    // Format sensor data into a single message
    String message = String(millis() / 1000.0, 1) + "," + 
                    String(bme.temperature, 1) + "," + 
                    String(bme.pressure / 100.0, 1) + "," + 
                    String(bme.humidity, 1) + "," + 
                    String(bme.gas_resistance / 1000.0, 1);
    
    sendMessage(message);
    Serial.println("SENT: " + message);
    lastSendTime = millis();            // timestamp the message
    interval = random(2000) + 1000;    // 2-3 seconds
  }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // Check for servo command
  if (incoming.startsWith("SERVO:")) {
    String angleStr = incoming.substring(6);  // Get the angle part after "SERVO:"
    int angle = angleStr.toInt();            // Convert to integer
    
    // Validate angle is within range
    if (angle >= 0 && angle <= 180) {
      myservo.write(angle);
      Serial.println("Servo set to: " + String(angle));
    } else {
      Serial.println("Invalid angle. Must be between 0 and 180.");
    }
    return;
  }

  // if message is for this device, or broadcast, print details:
  Serial.print("REC:" + incoming);
  Serial.println(", RSSI:" + String(LoRa.packetRssi()));
}

