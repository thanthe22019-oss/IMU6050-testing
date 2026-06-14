/* ============================================================================
 * ĐỀ TÀI: KHẢO SÁT ĐẶC TÍNH ĐỘNG HỌC VÀ ĐO LƯỜNG SANG CHẤN CƠ HỌC MPU-6050
 * Hạng mục: Thu thập dữ liệu tín hiệu gia tốc thời gian thực (Real-time DAQ)
 * * Mô tả thuật toán:
 * 1. Truy xuất dữ liệu vi cơ điện tử (MEMS) thông qua giao thức I2C.
 * 2. Biến đổi dữ liệu thô (Raw) sang đại lượng vật lý (g) qua Scale Factor.
 * 3. Áp dụng các hằng số bù trừ hệ thống (Zero-offset) từ thí nghiệm tĩnh.
 * 4. Tính toán cường độ vector gia tốc tổng (Euclidean Norm) trong không gian 3D.
 * 5. Xuất dữ liệu đồng bộ định kỳ với tần số lấy mẫu định mức Fs = 500Hz.
 * ============================================================================ */

#include <Wire.h>
#include <math.h>

// ----------------------------------------------------------------------------
// THÔNG SỐ CẤU HÌNH PHẦN CỨNG VÀ VẬT LÝ
// ----------------------------------------------------------------------------
const uint8_t MPU_ADDR = 0x68;                 // Địa chỉ I2C mặc định của MPU-6050
const float ACCEL_SCALE_FACTOR = 16384.0;      // Hệ số phân giải tương ứng dải đo định mức ±2g

// ----------------------------------------------------------------------------
// HẰNG SỐ HIỆU CHUẨN HỆ THỐNG (SYSTEM CALIBRATION CONSTANTS)
// (Các giá trị được kế thừa từ phương pháp hiệu chuẩn 6 vị trí - 6 Position Method)
// ----------------------------------------------------------------------------
const float OFFSET_AX = -0.0386;
const float OFFSET_AY = -0.0136;
const float OFFSET_AZ = -0.1360;

void setup() {
    // 1. Khởi tạo giao thức truyền thông nối tiếp (Baud rate cấu hình cao để tránh nghẽn cổ chai)
    Serial.begin(115200);

    // 2. Khởi tạo bus I2C và đánh thức vi mạch tích hợp
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);       // Trỏ tới thanh ghi Quản lý năng lượng (PWR_MGMT_1)
    Wire.write(0x00);       // Xóa bit Sleep, kích hoạt bộ dao động nội bộ (Internal Oscillator)
    Wire.endTransmission(true);

    // Xuất thông điệp giám sát trạng thái để xác nhận luồng thực thi
    Serial.println("[ HỆ THỐNG ] Tuyen trinh DAQ san sang. Tan so lay mau (Fs) = 500Hz.");
    delay(2000); // Thời gian chờ để linh kiện phần cứng đạt trạng thái ổn định nhiệt
}

void loop() {
    // ------------------------------------------------------------------------
    // GIAI ĐOẠN 1: ĐỌC TÍN HIỆU CẢM BIẾN (DATA ACQUISITION)
    // ------------------------------------------------------------------------
    // Định tuyến con trỏ thanh ghi đến vị trí lưu trữ dữ liệu gia tốc (ACCEL_XOUT_H)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    // Trích xuất liên tục 6 byte dữ liệu thô tương ứng với 3 trục tọa độ X, Y, Z
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);

    if (Wire.available() >= 6) {
        // Giải mã tín hiệu thô (Two's complement) thành số nguyên 16-bit
        int16_t raw_ax = (Wire.read() << 8) | Wire.read();
        int16_t raw_ay = (Wire.read() << 8) | Wire.read();
        int16_t raw_az = (Wire.read() << 8) | Wire.read();

        // --------------------------------------------------------------------
        // GIAI ĐOẠN 2: TIỀN XỬ LÝ TÍN HIỆU (SIGNAL CONDITIONING)
        // --------------------------------------------------------------------
        // Áp dụng phép biến đổi tuyến tính: Chuyển đổi dải đo và bù trừ sai số tĩnh
        float ax_calibrated = (raw_ax / ACCEL_SCALE_FACTOR) + OFFSET_AX;
        float ay_calibrated = (raw_ay / ACCEL_SCALE_FACTOR) + OFFSET_AY;
        float az_calibrated = (raw_az / ACCEL_SCALE_FACTOR) + OFFSET_AZ;

        // --------------------------------------------------------------------
        // GIAI ĐOẠN 3: PHÂN TÍCH ĐỘNG HỌC VÀ TRUYỀN TẢI (KINEMATIC ANALYSIS)
        // --------------------------------------------------------------------
        // Tính toán cường độ vector gia tốc tổng (Magnitude) để triệt tiêu ảnh hưởng của góc nghiêng
        float magnitude_vector = sqrt(pow(ax_calibrated, 2) + pow(ay_calibrated, 2) + pow(az_calibrated, 2));

        // Xuất tín hiệu với độ phân giải 4 chữ số thập phân, cung cấp dữ liệu đầu vào cho Python/FFT
        Serial.println(magnitude_vector, 4);
    }

    // ------------------------------------------------------------------------
    // KIỂM SOÁT THỜI GIAN THỰC (REAL-TIME TIMING CONTROL)
    // ------------------------------------------------------------------------
    // Cưỡng ép chu kỳ lấy mẫu (Sampling Period) T = 2ms, tương đương tần số Fs = 500Hz
    delay(2); 
}