#include <Wire.h>


// =====================================
// MPU6050 / MPU6500 I2C ADDRESS
// =====================================
const uint8_t MPU_ADDR = 0x68;


// =====================================
// SAMPLE CONFIG
// =====================================
const int MAX_SAMPLES = 500;
int counter = 0;


// =====================================
// RAW ACCEL DATA
// =====================================
int16_t raw_ax, raw_ay, raw_az;


void setup() {


  // Start Serial
  Serial.begin(115200);


  // Start I2C
  Wire.begin();


  Serial.println("=================================");
  Serial.println("MPU6050 / MPU6500 INITIALIZING...");
  Serial.println("=================================");


  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);       // PWR_MGMT_1
  Wire.write(0x00);       // Wake up
  byte status = Wire.endTransmission(true);


  // Check connection
  if (status != 0) {


    Serial.println("ERROR: MPU NOT DETECTED!");
    Serial.println("CHECK CONNECTION:");
    Serial.println("SDA -> A4");
    Serial.println("SCL -> A5");
    Serial.println("VCC -> 5V or 3.3V");
    Serial.println("GND -> GND");


    while (1);
  }


  Serial.println("MPU CONNECTED SUCCESSFULLY!");
  Serial.println();


  // Countdown
  for (int i = 5; i > 0; i--) {


    Serial.print("Starting in ");
    Serial.print(i);
    Serial.println(" second(s)");


    delay(1000);
  }


  Serial.println();
  Serial.println("=================================");
  Serial.println("Sample\tAccel_X\tAccel_Y\tAccel_Z");
  Serial.println("=================================");
}


void loop() {


  if (counter < MAX_SAMPLES) {


    // Start reading from ACCEL_XOUT_H
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);


    // Request 6 bytes
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);


    // Check enough bytes
    if (Wire.available() >= 6) {


      // Read RAW accelerometer data
      raw_ax = (Wire.read() << 8 | Wire.read());
      raw_ay = (Wire.read() << 8 | Wire.read());
      raw_az = (Wire.read() << 8 | Wire.read());


      counter++;


      // Print result
      Serial.print(counter);
      Serial.print("\t");


      Serial.print(raw_ax);
      Serial.print("\t");


      Serial.print(raw_ay);
      Serial.print("\t");


      Serial.println(raw_az);
    }


    // 50Hz
    delay(20);
  }


  else if (counter == MAX_SAMPLES) {


    Serial.println();
    Serial.println("=================================");
    Serial.println("500 RAW ACCEL SAMPLES COMPLETED");
    Serial.println("=================================");


    counter++;
  }
}
