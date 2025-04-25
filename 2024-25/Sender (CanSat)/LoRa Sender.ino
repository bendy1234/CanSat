#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <LoRa.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <MadgwickAHRS.h>
#include <RS-FEC.h> // https://github.com/simonyipeter/Arduino-FEC/tree/5d2164e6731d9f96e01aaf94d314d26e242f98e5
#include <FS.h>
#include <SD.h>

#define MADGWICK_TASK_INTERVAL 10

// I²C Addresses
#define BME_ADDR               0x76
#define MPU_ADDR               0x68 // or 0x69
#define QMC_ADDR               0x0D

// I²C Pins
#define SDA_PIN                8
#define SCL_PIN                9

// LoRa Config
#define LORA_FREQ              907E6
#define LORA_SYNC_WORD         0xF3
#define DATA_SEND_INTERVAL     1000
#define LORA_CS_PIN            10
#define RESET_PIN              40
#define IRQ_PIN                41

// SD card
#define SD_CS_PIN              45

// GPS Pins
#define GPS_RX_PIN             44
#define GPS_TX_PIN             43

// Servo Pins
#define AILERON_SERVO_PIN      42
#define UNLOCK_SERVO_PIN       38
// #define SERVO_PIN2             39

// QMC5883L Config
#define QMC_MODE_CONTINUOUS    (0b01)
#define QMC_ODR_50HZ           (0b10 << 2)
#define QMC_RANGE_2G           (0b00 << 4)
#define QMC_OSR_128            (0b10 << 6)

#define MAG_LSB_TO_UT          (100.0 / 12000.0)

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

RS::ReedSolomon<12, 6> shortRS;
RS::ReedSolomon<40, 8> longRS;
Madgwick filter;

// sensors
Adafruit_BME280 bme;
MPU6050 mpu;
QMC5883LCompass compass;
TinyGPSPlus gps;

// servos
Servo aileronServo;
Servo unlockServo;
// Servo servo4;

bool BYPASS_LORA = false;
float sea_level_pressure, max_altitude;

long lastSendTime, lastMadgwickRun;

Vec3f vel, pos, gps_cords, rot, mag;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CanSat!");
  
  filter.begin(100);
  initServos();
  initSensors();
  initLoRa();
  SD.begin(SD_CS_PIN);

  appendFile(SD, "out.txt", "========== PROGRAM START ==========");
  sea_level_pressure = bme.readPressure() / 100.0F;
}

void loop() {
  // check for incoming packets
  onReceive();

  if (millis() - lastSendTime > DATA_SEND_INTERVAL) {
    float data[10]; // TODO: figure out what to acc send
    data[0] = millis() / 1000.0;
    data[1] = bme.readTemperature();
    data[2] = bme.readPressure() / 100.0;
    data[3] = bme.readHumidity();
    data[4] = pos.x;
    data[5] = pos.y;
    data[6] = pos.z;
    data[7] = gps_cords.x;
    data[8] = gps_cords.y;
    data[9] = gps_cords.z; // altitude
    // data[10] = rot.x;
    // data[11] = rot.y;
    // data[12] = rot.z;
    float alt = bme.readAltitude(sea_level_pressure);
    if (max_altitude != -1 && alt > max_altitude) {
      max_altitude = alt;
    } else if ((max_altitude - alt) <= 50) {
      Serial.println("Auto unlock");
      appendFile(SD, "out.txt", "Auto unlock");
      servoCommand("unl0");
      max_altitude = -1;
    }

    lastSendTime = millis();
    sendMessage((char *) data, sizeof(data));
    
    File file = SD.open("out.txt", FILE_APPEND);
    if (!file) {
      Serial.println("Could not open file!");
      return;
    }

    file.print(data[0]);
    file.print("s, ");
    file.print(data[1]);
    file.print("∘c, ");
    file.print(data[2]);
    file.print( "Hpa, ");
    file.print(data[3]);
    file.print(", est pos: ");
    file.print(data[4]);
    file.print(", ");
    file.print(data[5]);
    file.print(", ");
    file.print(data[6]);
    file.print(", GPS: ");
    file.print(data[7]);
    file.print(", ");
    file.print(data[8]);
    file.print(", altitude: ");
    file.print(data[9]);
  
    file.print(", est rot: ");
    file.print(rot.x);
    file.print(", ");
    file.print(rot.y);
    file.print(", ");
    file.print(rot.z);
    file.print(", mag: ");
    file.print(mag.x);
    file.print(", ");
    file.print(mag.y);
    file.print(", ");
    file.print(mag.z);
    file.close();
  }
}

void initServos() {
  aileronServo.attach(AILERON_SERVO_PIN);
  unlockServo.attach(UNLOCK_SERVO_PIN);

  // set initial position to 90 degrees
  aileronServo.write(90);
  unlockServo.write(90);
}

void initSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);
  
  bool bme_init = !bme.begin(BME_ADDR, &Wire);
  
  mpu.initialize();
  bool mpu_init = !mpu.testConnection();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setXAccelOffset(-2169.0); mpu.setYAccelOffset(-281.0); mpu.setZAccelOffset(831.0);
  mpu.setXGyroOffset(35.0); mpu.setYGyroOffset(-17.0); mpu.setZGyroOffset(26.0);
  mpu.setI2CBypassEnabled(true);

  compass.setADDR(QMC_ADDR);
  compass.init();
  compass.setMode(QMC_MODE_CONTINUOUS, QMC_ODR_50HZ, QMC_RANGE_2G, QMC_OSR_128);
  compass.setCalibrationOffsets(-4641.00, -1958.00, 3594.00);
  compass.setCalibrationScales(1.16, 1.16, 0.78);

  // GPS
  Serial2.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  xTaskCreate(GPSTask, "GPS", 4096, NULL, 2, NULL);
  xTaskCreate(rotationTask, "rotation", 4096, NULL, 3, NULL);

  if (bme_init || mpu_init) {
    Serial.println("Could not find the following sensor(s):");
    if (bme_init) Serial.println("BME280");
    if (mpu_init) Serial.println("MPU6050");
    // TODO: compass
    while (true);
  }
}

void initLoRa() {
  if (BYPASS_LORA) {Serial.println("WARNING: LoRa bypass enabled"); return;}
  
  LoRa.setPins(LORA_CS_PIN, RESET_PIN, IRQ_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed. Check your connections.");
    BYPASS_LORA = true;
  }

  LoRa.setSyncWord(LORA_SYNC_WORD);
  Serial.println("LoRa init succeeded.");
}


void rotationTask(void* _) {
  while (true) {
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    compass.read();
    mx = compass.getX(); my = compass.getY(); mz = compass.getZ();

    // FIXME: Magic numbers
    Vec3f acc = {
      float(ax) * 2.0 / 32768.0,
      float(ay) * 2.0 / 32768.0,
      float(az) * 2.0 / 32768.0,
    };

    mag = {
      mx * MAG_LSB_TO_UT,
      my * MAG_LSB_TO_UT,
      mz * MAG_LSB_TO_UT
    };

    filter.update(acc.x, acc.y, acc.z,
                  gx * 250.0 / 32768.0, gy * 250.0 / 32768.0, gz * 250.0 / 32768.0,
                  mag.x, mag.y, mag.z);
    float dt = (millis() - lastMadgwickRun) / 1000.0;
    lastMadgwickRun = millis();

    rot = {
      filter.getRoll(),
      filter.getPitch(),
      filter.getYaw(),
    };
    
    // NOTE: Added in MadgwickAHRS.h:
    // void getQuaternion(float* _q0, float* _q1, float* _q2, float* _q3) const {
    //     *_q0 = q0; *_q1 = q1; *_q2 = q2; *_q3 = q3;
    // }
    // Why? cuz doing lots of trig is slow

    float q0, q1, q2, q3;
    filter.getQuaternion(&q0, &q1, &q2, &q3);
    acc = rotateVec3ByQuaternion(acc.x, acc.y, acc.z, q0, q1, q2, q3);
    vel += acc * dt;
    pos += vel * dt;

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void GPSTask(void* _) {
  while (true) {
    while (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        gps_cords = {gps.location.lat(), gps.location.lng(), gps.altitude.meters()};
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
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
  } else if (command.startsWith("unl")) {
    unlockServo.write(angle);
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
  appendFile(SD, "out.txt", ("SERVO:" + command).c_str());

  delay(10);
  sendMessage("SERVO:" + command);
}


void sendMessage(String msg) {
  if (BYPASS_LORA) return;
  sendMessage(msg.c_str(), msg.length());
  Serial.println("SENT: " + msg);
}

void sendMessage(const char* msg, uint8_t lenght) {
  if (BYPASS_LORA) return;

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
  LoRa.receive();
}

void onReceive() {
  if (BYPASS_LORA) return;
  if (LoRa.parsePacket() == 0) return;

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

Vec3f rotateVec3ByQuaternion(float x, float y, float z, float q0, float q1, float q2, float q3) {
  // Perform quaternion multiplication: q' = q * v * q^-1
  
  float qv0 = -q1 * x - q2 * y - q3 * z;
  float qv1 = q0 * x + q2 * z - q3 * y;
  float qv2 = q0 * y - q1 * z + q3 * x;
  float qv3 = q0 * z + q1 * y - q2 * x;

  float rotX = qv0 * q0 - qv1 * -q1 - qv2 * -q2 - qv3 * -q3;
  float rotY = qv0 * -q1 + qv1 * q0 + qv2 * -q3 - qv3 * -q2;
  float rotZ = qv0 * -q2 - qv1 * -q3 + qv2 * q0 + qv3 * -q1;

  return Vec3f{rotX, rotY, rotZ};
} 

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Could not open file!");
    return;
  }
  if (file.print(message)) {
  } else {
    Serial.println("Could not write to file!");
  }
  file.close();
}
