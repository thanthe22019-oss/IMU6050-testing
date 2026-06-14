#include <Wire.h>
#include <math.h>

// =========================================================================
// THÔNG SỐ VẬT LÝ VÀ HỆ SỐ QUY ĐỔI (SCALE FACTORS)
// =========================================================================
const uint8_t MPU_ADDR = 0x68;           // Địa chỉ I2C mặc định của nền tảng MPU-6050/6500
const int CALIBRATION_SAMPLES = 500;     // Kích thước mẫu định mức (Quần thể 500 mẫu)
const float GYRO_SCALE_FACTOR = 131.0;   // Hệ số phân giải độ nhạy cho dải đo ±250 °/s
const float STD_TOLERANCE = 0.3;        // Ngưỡng dung sai độ lệch chuẩn tối đa cho phép

// =========================================================================
// BIẾN TOÀN CỤC LƯU TRỮ HẰNG SỐ BÙ TRỪ (ZERO-OFFSET)
// =========================================================================
float offset_gx = 0.0, offset_gy = 0.0, offset_gz = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Khởi tạo trạng thái hoạt động của cảm biến
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);       // Truy xuất thanh ghi PWR_MGMT_1
  Wire.write(0x00);       // Hủy chế độ Sleep, kích hoạt bộ dao động nội (Internal Oscillator)
  Wire.endTransmission(true);

  Serial.println("\n[ THÔNG BÁO ] KÍCH HOẠT TIẾN TRÌNH HIỆU CHUẨN ĐỘNG HỌC (GYROSCOPE ONLY)");
  Serial.println("[ HƯỚNG DẪN ] Yêu cầu duy trì thiết bị ở trạng thái cô lập dao động cơ học hoàn toàn.");
  
  bool isCalibrated = false;

  while (!isCalibrated) {
    Serial.println("\n[ TIẾN TRÌNH ] Đang tiến hành trích xuất mẫu kiểm định (Thời gian ước tính: 5s)...");
    
    // Khởi tạo các biến tích lũy cho Thuật toán thống kê Welford
    float mean_gx = 0.0, mean_gy = 0.0, mean_gz = 0.0;
    float M2_gx = 0.0, M2_gy = 0.0, M2_gz = 0.0;

    for (int i = 1; i <= CALIBRATION_SAMPLES; i++) {
      Wire.beginTransmission(MPU_ADDR);
      // Bỏ qua vùng nhớ Accelerometer, truy xuất trực tiếp vào thanh ghi GYRO_XOUT_H
      Wire.write(0x43); 
      Wire.endTransmission(false);
      
      // Yêu cầu tải chuỗi 6 byte dữ liệu động học (X, Y, Z)
      Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);

      if (Wire.available() >= 6) {
        // Trích xuất và giải mã tín hiệu thô (Raw Data) theo chuẩn Two's Complement
        int16_t raw_gx = (Wire.read() << 8) | Wire.read();
        int16_t raw_gy = (Wire.read() << 8) | Wire.read();
        int16_t raw_gz = (Wire.read() << 8) | Wire.read();

        // Áp dụng phép biến đổi tuyến tính sang hệ đo lường chuẩn (°/s)
        float gx = raw_gx / GYRO_SCALE_FACTOR;
        float gy = raw_gy / GYRO_SCALE_FACTOR;
        float gz = raw_gz / GYRO_SCALE_FACTOR;

        // Thực thi thuật toán Welford tính toán giá trị kỳ vọng (Mean) và phương sai (Variance)
        float delta_x = gx - mean_gx; mean_gx += delta_x / i; M2_gx += delta_x * (gx - mean_gx);
        float delta_y = gy - mean_gy; mean_gy += delta_y / i; M2_gy += delta_y * (gy - mean_gy);
        float delta_z = gz - mean_gz; mean_gz += delta_z / i; M2_gz += delta_z * (gz - mean_gz);
      }
      delay(10); // Tần số lấy mẫu: 100Hz (Chu kỳ 10ms)
    }

    // Đánh giá chỉ tiêu nhiễu tĩnh (Độ lệch chuẩn - Standard Deviation)
    float std_gx = sqrt(M2_gx / CALIBRATION_SAMPLES);
    float std_gy = sqrt(M2_gy / CALIBRATION_SAMPLES);
    float std_gz = sqrt(M2_gz / CALIBRATION_SAMPLES);

    Serial.println("[ KẾT QUẢ ĐO LƯỜNG ] Biên độ nhiễu tín hiệu (STD):");
    Serial.print("  - Trục X: "); Serial.print(std_gx, 4); Serial.println(" °/s");
    Serial.print("  - Trục Y: "); Serial.print(std_gy, 4); Serial.println(" °/s");
    Serial.print("  - Trục Z: "); Serial.print(std_gz, 4); Serial.println(" °/s");

    // Thẩm định điều kiện hội tụ của thuật toán dựa trên dung sai định mức
    if (std_gx <= STD_TOLERANCE && std_gy <= STD_TOLERANCE && std_gz <= STD_TOLERANCE) {
      offset_gx = mean_gx;
      offset_gy = mean_gy;
      offset_gz = mean_gz;
      
      Serial.println("\n[ THÀNH CÔNG ] Trạng thái tĩnh đạt chuẩn. Các hằng số bù trừ đã được xác lập:");
      Serial.print("  + Offset Gyro X: "); Serial.println(offset_gx, 4);
      Serial.print("  + Offset Gyro Y: "); Serial.println(offset_gy, 4);
      Serial.print("  + Offset Gyro Z: "); Serial.println(offset_gz, 4);
      
      isCalibrated = true; // Giải phóng tiến trình hiệu chuẩn
    } else {
      Serial.println("\n[ CẢNH BÁO ] Phát hiện dao động ngoại biên vượt ngưỡng cho phép (STD > 0.3).");
      Serial.println("Hệ thống sẽ tự động tái khởi động chu trình hiệu chuẩn sau 2 giây...");
      delay(2000);
    }
  }
}

void loop() {
  // Không gian thực thi vòng lặp thuật toán chính
}