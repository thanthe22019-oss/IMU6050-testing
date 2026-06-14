#include <Wire.h>
#include <math.h>

const uint8_t MPU_ADDR = 0x68;
const int NUM_SAMPLES = 200;
const float ACCEL_SCALE = 16384.0;

// ==========================================================
// HẰNG SỐ BÙ TRỪ CÁ NHÂN HÓA (Đã tính từ 6-position method)
// ==========================================================
const float OFFSET_AX = -0.0386;
const float OFFSET_AY = -0.0136;
const float OFFSET_AZ = -0.1360;

// Các góc nghiêng cần khảo sát theo kịch bản
const int test_angles[] = {0, 15, 30, 45, 60, 90};
const int num_tests = 6;

// Giá trị lý thuyết (dựa trên hàm lượng giác g*sin(theta) và g*cos(theta))
const float theory_ax[] = {0.000, 0.259, 0.500, 0.707, 0.866, 1.000};
const float theory_az[] = {1.000, 0.966, 0.866, 0.707, 0.500, 0.000};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);       
  Wire.write(0x00);       
  Wire.endTransmission(true);

  Serial.println("\n=======================================================");
  Serial.println("KHAO SAT SU PHAN BO VECTOR GIA TOC TRONG TRUONG (g)");
  Serial.println("=======================================================\n");

  for (int i = 0; i < num_tests; i++) {
    int angle = test_angles[i];
    
    // YÊU CẦU ĐIỀU KIỆN CHUẨN TRƯỚC KHI ĐO
    Serial.print("\n[ GOC "); Serial.print(angle); Serial.println(" DO ]");
    Serial.println("-> YEU CAU: Dung e-ke/thuoc do goc ke mach nghieng DUNG " + String(angle) + " do (truc Y co dinh, nang truc X).");
    Serial.println("-> Vui long kiem tra ky bang mat thuong. Go phi'm bat ky roi Enter de tien hanh do...");
    
    waitForKey(); // Chờ người dùng xác nhận đã kê chuẩn

    // Trích xuất dữ liệu trung bình sau 200 mẫu
    float mean_ax, mean_ay, mean_az;
    measureAccel(mean_ax, mean_ay, mean_az);

    // Kiểm tra toàn diện vector
    float vector_sum_sq = pow(mean_ax, 2) + pow(mean_ay, 2) + pow(mean_az, 2);
    float deviation = abs(vector_sum_sq - 1.0) * 100.0; // Tính % sai lệch

    // BÁO CÁO KẾT QUẢ ĐỐI CHIẾU
    Serial.println("-------------------------------------------------------");
    Serial.println("                     | Ly thuyet      | Thuc te đo được");
    Serial.print("Truc X (sin(theta))  | "); Serial.print(theory_ax[i], 3); Serial.print("          | "); Serial.println(mean_ax, 3);
    Serial.print("Truc Y (co dinh)     | 0.000          | "); Serial.println(mean_ay, 3);
    Serial.print("Truc Z (cos(theta))  | "); Serial.print(theory_az[i], 3); Serial.print("          | "); Serial.println(mean_az, 3);
    
    Serial.print("\nKiem tra toan dien: Ax^2 + Ay^2 + Az^2 = "); 
    Serial.print(vector_sum_sq, 4); Serial.println(" g");
    
    Serial.print("Do lech vector: "); Serial.print(deviation, 2); Serial.println(" %");
    
    if (deviation > 3.0) {
      Serial.println("[ CANH BAO ] Sai lech > 3%! Can hieu chinh lai gain (ti le) hoac kiem tra lai the ke mach.");
    } else {
      Serial.println("[ DAT CHUAN ] He thong phan bo luc tuyen tinh hoan hao.");
    }
    Serial.println("-------------------------------------------------------\n");
  }
  
  Serial.println(">>> DA HOAN THANH TOAN BO BAI KIEM TRA! <<<");
}

void loop() {
}

// Hàm chờ người dùng xác nhận
void waitForKey() {
  while (Serial.available() > 0) Serial.read(); 
  while (Serial.available() == 0) { delay(10); } 
  while (Serial.available() > 0) Serial.read(); 
  Serial.println(">> DANG DO 200 MAU... GIK IM MACH!");
}

// Hàm đo đạc đã tích hợp Offset cá nhân hóa
void measureAccel(float &ax_out, float &ay_out, float &az_out) {
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6);

    if (Wire.available() >= 6) {
      int16_t raw_ax = (Wire.read() << 8) | Wire.read();
      int16_t raw_ay = (Wire.read() << 8) | Wire.read();
      int16_t raw_az = (Wire.read() << 8) | Wire.read();

      // Áp dụng công thức: (Raw / Scale) + Offset
      float ax = (raw_ax / ACCEL_SCALE) + OFFSET_AX;
      float ay = (raw_ay / ACCEL_SCALE) + OFFSET_AY;
      float az = (raw_az / ACCEL_SCALE) + OFFSET_AZ;

      sum_ax += ax; sum_ay += ay; sum_az += az;
    }
    delay(5);
  }
  
  ax_out = sum_ax / NUM_SAMPLES;
  ay_out = sum_ay / NUM_SAMPLES;
  az_out = sum_az / NUM_SAMPLES;
}