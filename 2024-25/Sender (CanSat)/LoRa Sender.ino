#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADXL343.h>
#include <ESP32Servo.h>

// pins
#define SDA_PIN 8
#define SCL_PIN 9

// LoRa radio chip select
#define csPin 10
#define resetPin 40
#define irqPin 41


#define BME_ADDR 0x76
#define ADXL_ADDR 0x53

#define SEALEVELPRESSURE_HPA 1000.0

#define aileronServoPin 38
// #define servoPin2 39
#define rudderServoPin 42
// #define servoPin4 45

// simple vec3 for what i need
struct Vec3f {
  float x, y, z;
  Vec3f operator*(float scalar) const {
    return {x * scalar, y * scalar, z * scalar};
  }

  Vec3f& operator*=(float scalar) {
    x *= scalar; y *= scalar; z *= scalar;
    return *this;
  }

  Vec3f& operator+=(const Vec3f& other) {
    x += other.x; y += other.y; z += other.z;
    return *this;
  }
};

// sensors
Adafruit_BME280 bme;
Adafruit_ADXL343 accel = Adafruit_ADXL343(ADXL_ADDR, &Wire);

// servos
Servo aileronServo;
// Servo servo2;
Servo rudderServo;
// Servo servo4;

byte msgCount = 0;

long lastSendTime, lastAccelReadTime;
int interval = 2000;

Vec3f vel, pos;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CanSat!");

  // Initialize servos
  aileronServo.attach(aileronServoPin);
  rudderServo.attach(rudderServoPin);

  // set initial position to 90 degrees
  aileronServo.write(90);
  rudderServo.write(90);

  // Initialize BME280 sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bme.begin(BME_ADDR, &Wire)) {
    Serial.println("Could not find a valid BME280 sensor!");
    while (true);
  }

  if (!accel.begin()) {
    Serial.println("Could not find ADXL343 sensor!");
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
  // check for incoming packets
  onReceive(LoRa.parsePacket());

  // TODO: use interrupts if possible
  // always read accelerometer data when possible
  int16_t x, y, z;
  if (accel.getXYZ(x, y, z)) {
    // integrate forces
    float interval = (millis() - lastAccelReadTime) / 1000.0;
    Vec3f acceleration = Vec3f{x, y, z} * (ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD * interval);
    
    vel += acceleration;
    pos += vel * interval;

    lastAccelReadTime = millis();
  }

  if (millis() - lastSendTime > interval) {
    char msg[128];
    snprintf(msg, sizeof(msg), "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
              millis() / 1000.0,
              bme.readTemperature(),
              bme.readPressure() / 100.0,
              bme.readHumidity(),
              vel.x,
              vel.y,
              vel.z,
              pos.x,
              pos.y,
              pos.z
    );
    
    sendMessage(msg);
    lastSendTime = millis();
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
