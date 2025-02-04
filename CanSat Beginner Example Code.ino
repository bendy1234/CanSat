#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Pin definitions
#define SD_CS_PIN 10 // Chip select pin for the SD card

// BMP280 object
Adafruit_BMP280 bmp;

// File object for the SD card
File dataFile;

// Timing variables
unsigned long previousMillis = 0;
const long interval = 250; // Log data every 250 ms
unsigned long startTime = 0; // Store the start time

// Global variable to store the filename
char filename[15];

// Variable for calibrated sea level pressure
float seaLevelPressure;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Create a unique filename based on timestamp
  int fileNumber = 0;
  do {
    snprintf(filename, sizeof(filename), "log_%03d.csv", fileNumber++);
  } while (SD.exists(filename));

  // Open the new log file
  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Failed to open file for writing!");
    while (1);
  }
  Serial.print("Logging to file: ");
  Serial.println(filename);

  // Initialize BMP280 sensor
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  Serial.println("BMP280 sensor initialized.");

  // Calibrate sea level pressure based on current altitude (assume 0 altitude at start)
  seaLevelPressure = bmp.readPressure() / 100.0F; // Convert to hPa
  Serial.print("Calibrated sea level pressure: ");
  Serial.print(seaLevelPressure);
  Serial.println(" hPa");

  // Write CSV header to the file
  dataFile.println("Time(s),Temperature(C),Pressure(hPa),Altitude(m)");
  dataFile.close();

  // Record the start time
  startTime = millis();
}

void loop() {
  // Get current time
  unsigned long currentMillis = millis();

  // Log data at specified intervals
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calculate time since start in seconds with fractional precision
    float timeSinceStart = (currentMillis - startTime) / 1000.0;

    // Variables to store averaged values
    float avgTemperature = 0;
    float avgPressure = 0;
    float avgAltitude = 0;

    // Measure 4 times and calculate the average
    for (int i = 0; i < 4; i++) {
      avgTemperature += bmp.readTemperature();
      avgPressure += bmp.readPressure() / 100.0F; // Convert to hPa
      avgAltitude += bmp.readAltitude(seaLevelPressure); // Use calibrated sea level pressure
      delay(62); // Wait 62 ms between readings to fit within 250 ms total
    }

    avgTemperature /= 4;
    avgPressure /= 4;
    avgAltitude /= 4;

    // Print data to serial monitor
    Serial.print("Time since start: ");
    Serial.print(timeSinceStart, 2); // Print with 2 decimal places
    Serial.print(" s, Temperature: ");
    Serial.print(avgTemperature);
    Serial.print(" °C, Pressure: ");
    Serial.print(avgPressure);
    Serial.print(" hPa, Altitude: ");
    Serial.print(avgAltitude);
    Serial.println(" m");

    // Write data to the CSV file
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      dataFile.print(timeSinceStart, 2); // Log with 2 decimal places
      dataFile.print(",");
      dataFile.print(avgTemperature);
      dataFile.print(",");
      dataFile.print(avgPressure);
      dataFile.print(",");
      dataFile.println(avgAltitude);
      dataFile.close();
    } else {
      Serial.println("Failed to open file for writing!");
    }
  }
}
