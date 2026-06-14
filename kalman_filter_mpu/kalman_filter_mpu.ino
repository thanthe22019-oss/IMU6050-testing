#include <Wire.h>
#include <math.h>

const uint8_t MPU_ADDR = 0x68;

const float ACCEL_SCALE = 16384.0; 
const float GYRO_SCALE = 131.0;    

// Offsets - Thay bằng số đo của riêng con 6050 của bạn nhé
const float OFFSET_AX = -0.0386;
const float OFFSET_AY = -0.0136;
const float OFFSET_AZ = -0.1360;

const float OFFSET_GX = -1.9811;
const float OFFSET_GY = 0.4820;
const float OFFSET_GZ = -0.7317;

uint32_t timer;

float roll_kalman = 0.0; 
float roll_bias = 0.0;   
float P_roll[2][2] = {{1.0, 0.0}, {0.0, 1.0}}; 

float pitch_kalman = 0.0; 
float pitch_bias = 0.0;   
float P_pitch[2][2] = {{1.0, 0.0}, {0.0, 1.0}}; 

// =========================================================================
// ĐƠN THUỐC MỚI CHO BỆNH NHÂN MPU6050
// =========================================================================
// Vì STD Gyro lên tới 0.25 (rất nhiễu), ta tăng nhẹ Q_angle để hệ thống 
// bớt bảo thủ với mô hình dự báo của Gyro.
const float Q_angle = 0.002;   

// Gyro của 6050 trôi (drift) rất gắt, cần tăng tốc độ gột rửa định kiến lên
const float Q_bias = 0.005;    

// Gia tốc của 6050 cũng ồn ào không kém. Tăng R_measure từ 0.03 lên 0.1 
// để bác sĩ Kalman rộng lượng "ngó lơ" bớt sự giật cục của cô vợ Accel.
const float R_measure = 0.1;  

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);       
  Wire.write(0x00);       
  Wire.endTransmission(true);

  // ÉP UỐNG THUỐC AN THẦN PHẦN CỨNG (DLPF - Digital Low Pass Filter)
  // Can thiệp vào thanh ghi 0x1A. Đặt giá trị 0x05 để giới hạn băng thông 
  // của Accel ở mức 10Hz và Gyro ở 10Hz. Lọc ngay từ trong trứng nước!
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05); 
  Wire.endTransmission(true);

  Serial.println("Da thiet lap xong bo loc cho MPU6050...");
  delay(1000);
  timer = micros(); 
}

float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
  float rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float y = newAngle - angle; 
  float S = P[0][0] + R_measure; 
  
  float K_angle = P[0][0] / S;
  float K_bias = P[1][0] / S;

  angle += K_angle * y;
  bias += K_bias * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K_angle * P00_temp;
  P[0][1] -= K_angle * P01_temp;
  P[1][0] -= K_bias * P00_temp;
  P[1][1] -= K_bias * P01_temp;

  return angle; 
}

void loop() {
  float dt = (float)(micros() - timer) / 1000000.0;
  timer = micros();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

  if (Wire.available() >= 14) {
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); 
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    float ax = (raw_ax / ACCEL_SCALE) + OFFSET_AX;
    float ay = (raw_ay / ACCEL_SCALE) + OFFSET_AY;
    float az = (raw_az / ACCEL_SCALE) + OFFSET_AZ;

    float gx = (raw_gx / GYRO_SCALE) - OFFSET_GX;
    float gy = (raw_gy / GYRO_SCALE) - OFFSET_GY;

    float roll_accel = atan2(ay, az) * 180.0 / M_PI;
    float pitch_accel = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

    float roll_final = kalmanFilter(roll_accel, gx, dt, roll_kalman, roll_bias, P_roll);
    float pitch_final = kalmanFilter(pitch_accel, gy, dt, pitch_kalman, pitch_bias, P_pitch);

    Serial.print("Roll_Raw:"); Serial.print(roll_accel); Serial.print(",");
    Serial.print("Roll_Kalman:"); Serial.println(roll_final); Serial.print(",");
    Serial.print("Pitch_Raw:"); Serial.print(pitch_accel); Serial.print(",");
    Serial.print("Pitch_Kalman:"); Serial.println(pitch_final);
  }
}