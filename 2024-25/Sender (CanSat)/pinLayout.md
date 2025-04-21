# Pin layout

## Lora:

SS = 10  
MOSI = 11  
SCK = 12  
MISO = 13  
RST = 40  
DIO0 = 41  

## I²C

SDA = 8  
SCL = 9  

| Sensor   | Address |
| :------: | :-----: |
| BMP 280  |   0x76  |
| ADXL343  |   0x53  |
| MPU6050  |   0x68  |
| QMC5883L |   0x0D  |

Note, the QMC5883L conected through the MPU (XDA, XCL pins)

## Servos

Signal -> 38, 39, 42, 45
