void parseData(uint8_t* buffer, int bytesRead) {
  char type = buffer[0];
  if (type != 'B' && type != 'G') {
    Serial.print("Unknown packet type: '");
    Serial.print(type);
    if ((bytesRead - 3) == sizeof(BMEData)) {
      Serial.println("' Expecting BME data");
      type = 'B';
    } else if ((bytesRead - 3) == sizeof(GPSData)) {
      Serial.println("' Expecting GPS data");
      type = 'G';
    } else {
      Serial.println("' Skipping!");
      return;
    }
  }

  uint16_t packetNumber;
  memcpy(&packetNumber, buffer + 1, sizeof(uint16_t));
  Serial.print("Packet # ");
  Serial.print(packetNumber);
  Serial.print(": ");

  // no need to worry about memory isuses
  if (type == 'B') {
    BMEData data;
    memcpy(&data, buffer + 3, sizeof(BMEData));
    Serial.print(data.temperature);
    Serial.print("°C, ");
    Serial.print(data.pressure);
    Serial.print("hPa, ");
    Serial.print(data.humidity);
    Serial.println("%");
  } else if (type == 'G') {
    GPSData data;
    memcpy(&data, buffer + 3, sizeof(GPSData));
    Serial.print("Lat: ");
    Serial.print(data.latitude);
    Serial.print("°, Lon: ");
    Serial.print(data.longitude);
    Serial.print("°, Alt: ");
    Serial.print(data.altitude / 100.0);
    Serial.print("m, Date: ");
    Serial.print(data.date); // ddmmyy
    Serial.print(", Time: ");
    Serial.println(data.time); // hhmmsscc
  }
}

void setup() {
  Serial.begin(115200);

  // set up the test data
  uint16_t count = 1;
  uint8_t buffer[3 + sizeof(GPSData)]; // 3 bytes header + packet number

  BMEData bmeData{ 25.3f, 1013.2f, 80.5f };
  buffer[0] = 'B';
  memcpy(buffer + 1, &count, sizeof(uint8_t));
  memcpy(buffer + 3, &bmeData, sizeof(BMEData));

  Serial.println("1) Writing test BME data");
  parseData(buffer, 3 + sizeof(BMEData));

  // Now for the GPS data
  GPSData gpsData{ 10.0f, 20.0f, 300, 1234567, 123456 };

  buffer[0] = 'G';
  count++;
  memcpy(buffer + 1, &count, sizeof(uint8_t));
  memcpy(buffer + 3, &gpsData, sizeof(GPSData));

  Serial.println("2) Writing test GPS data");
  parseData(buffer, 3 + sizeof(GPSData));

  // now test corruption handling
  // NOTE: the only checks are on the header and the size of the data
  // so there is no use in messing with the other bits

  Serial.println("3) Wrong header, but correct size for GPS");
  buffer[0] = 'a';

  count++;
  memcpy(buffer + 1, &count, sizeof(uint8_t));
  // reusing the previous data

  parseData(buffer, 3 + sizeof(GPSData));

  Serial.println("4) Wrong header, but correct size for BME");
  buffer[0] = 'a';
  count++;
  memcpy(buffer + 1, &count, sizeof(uint8_t));
  memcpy(buffer + 3, &bmeData, sizeof(BMEData));

  parseData(buffer, 3 + sizeof(BMEData));

  Serial.println("5) Wrong header, wrong size");
  buffer[0] = 'a';
  count++;
  memcpy(buffer + 1, &count, sizeof(uint8_t));
  for (int i = 3; i < (3 + sizeof(BMEData)); i++) {
    buffer[i] = buffer[i + 1];
  }

  parseData(buffer, 2 + sizeof(BMEData));

  Serial.println("6) Right header, wrong size (missing byte)");
  buffer[0] = 'B';
  count++;
  memcpy(buffer + 1, &count, sizeof(uint8_t));

  // simulate missing a byte by shifting the data
  for (int i = 3; i < (3 + sizeof(BMEData)); i++) {
    buffer[i] = buffer[i + 1];
  }

  parseData(buffer, 2 + sizeof(BMEData)); // 2 for missing byte

  Serial.println("7) Random data");
  for (int i = 0; i < (3 + sizeof(GPSData)); i++) {
    buffer[i] = i << 1;
  }
  parseData(buffer, 3 + sizeof(GPSData));
}
