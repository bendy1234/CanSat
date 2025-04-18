#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADXL343.h>
#include <ESP32Servo.h>
#include <RS-FEC.h> // https://github.com/simonyipeter/Arduino-FEC/tree/5d2164e6731d9f96e01aaf94d314d26e242f98e5

#define BYPASS_LORA true

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

RS::ReedSolomon<12, 6> shortRS;
RS::ReedSolomon<40, 8> longRS;

uint8_t msgCount = 0;

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

  if (!BYPASS_LORA) {
    // override the default CS, reset, and IRQ pins (optional)
    LoRa.setPins(csPin, resetPin, irqPin);

    if (!LoRa.begin(907E6)) {             // initialize ratio at 907 MHz
      Serial.println("LoRa init failed. Check your connections.");
      while (true);
    }

    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa init succeeded.");
  } else {
    Serial.println("WARNING: LoRa bypass enabled");
  }
}

void loop() {
  // check for incoming packets
  if (!BYPASS_LORA) {
    onReceive(LoRa.parsePacket());
  }

  // TODO: use interrupts if possible
  // always read accelerometer data when possible
  int16_t x, y, z;
  if (accel.getXYZ(x, y, z)) {
    // integrate forces
    float time_passed = (millis() - lastAccelReadTime) / 1000.0;
    Vec3f acceleration = Vec3f{x, y, z} * (ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD * time_passed);
    
    vel += acceleration;
    pos += vel * time_passed;

    lastAccelReadTime = millis();
  }

  if (millis() - lastSendTime > interval) {
    float data[10];
    data[0] = millis() / 1000.0;
    data[1] = bme.readTemperature();
    data[2] = bme.readPressure() / 100.0;
    data[3] = bme.readHumidity();
    data[4] = vel.x;
    data[5] = vel.y;
    data[6] = vel.z;
    data[7] = pos.x;
    data[8] = pos.y;
    data[9] = pos.z;

    lastSendTime = millis();
    if (!BYPASS_LORA) {
      sendMessage((char *) data, sizeof(data));
      LoRa.receive();
    }

    Serial.print(data[0]);
    Serial.print("s, ");
    Serial.print(data[1]);
    Serial.print("∘c, ");
    Serial.print(data[2]);
    Serial.print( "Hpa, ");
    Serial.print(data[3]);
    Serial.print("%, vel: ");
    Serial.print(data[4]);
    Serial.print(", ");
    Serial.print(data[5]);
    Serial.print(", ");
    Serial.print(data[6]);
    Serial.print(", est pos: ");
    Serial.print(data[7]);
    Serial.print(", ");
    Serial.print(data[8]);
    Serial.print(", ");
    Serial.println(data[9]);
  }
}

void servoCommand(String command) {
  int angle = command.substring(3).toInt();
  // Validate angle
  if (angle < 0 || angle > 180) {
    Serial.println("Invalid servo angle");
    sendMessage("SERVO:BadAng");
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
    sendMessage("SERVO:Unknwn");
    return;
  }

  Serial.print("Servo ");
  Serial.print(command.substring(0, 2));
  Serial.println(" set to: " + String(angle));

  delay(10);
  sendMessage("SERVO:" + command);
}

void sendMessage(String msg) {
  sendMessage(msg.c_str(), msg.length());
  Serial.println("SENT: " + msg);
}

void sendMessage(const char* msg, uint8_t lenght) {
  LoRa.beginPacket();
  if (lenght > 12) {
    uint8_t out[40 + 8]; // msg should never be longer than 40 chars
    longRS.Encode(msg, out);
    LoRa.write(out, sizeof(out));
  } else {
    uint8_t out[12 + 6];
    shortRS.Encode(msg, out);
    LoRa.write(out, sizeof(out));
  }
  LoRa.endPacket();
  msgCount++;
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  // get message
  char data[18], out[13];
  out[12] = 0;
  
  LoRa.readBytes(data, sizeof(data));
  shortRS.Decode(data, out);
  String msg = String(out);

  Serial.print("RECV: ");
  Serial.println(msg);

  // Check for servo command
  if (msg.startsWith("SERVO:")) {
    servoCommand(msg.substring(6));
  }
}
