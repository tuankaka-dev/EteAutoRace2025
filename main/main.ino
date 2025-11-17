#define PWMA 5
#define PWMB 6
#define AIN1 8
#define AIN2 9
#define BIN1 10
#define BIN2 11
#define STBY 7  // bật mạch TB6612FNG

#define SW 2
#define TRIG 12
#define ECHO 13

// ================== THAM SỐ CƠ BẢN & PID ==================
const int BASE = 255;
const int PWM_MAX = 255;

const int Kp = 32, Ki = 0, Kd = 3;
int P, I = 0, D;
int previous_error = 0;
int previous_error_map = 0;
int error = 0, PIDValue = 0;
int TN = 400;  // ngưỡng line

// ================== CP1: FLAG START ==================
const int START_BLOCK_CM = 8;                   // bị chặn nếu khoảng cách <= 8 cm
const unsigned long START_CLEAR_HOLD_MS = 300;  // thông thoáng liên tục >= 300 ms -> chạy
bool startflag_done = false;

// ================== CP2: CROSS LINE ==================
int cross_count = 0;
// ================== CP3: Lost line ==================
int lost_count = 0;
unsigned long lost_since = 0;   // Thời điểm bắt đầu mất line
unsigned long found_since = 0;  // Thời điểm bắt đầu tìm thấy line
bool lost_line_confirmed = false;
const unsigned long LOST_LINE_THRESHOLD_MS = 22;   // Ngưỡng mất line để tăng lost_count
const unsigned long FOUND_LINE_THRESHOLD_MS = 22;  // Ngưỡng tìm thấy line để reset lost_count

// ================== CP4: OBSTACLE AVOID (sau khi đã xuất phát) ==================
const int OBSTACLE_THRESHOLD_CM = 15;         // <=10cm thì coi là vật cản
const unsigned long OBST_BLOCK_HOLD_MS = 40;  // bị chặn liên tục >=80ms mới né
const unsigned long ULTRA_PERIOD_MS = 60;     // chu kỳ đo siêu âm
bool avoid_done = false;
unsigned long lastUltraCheck = 0;
unsigned long block_since = 0;
bool obstacle_confirmed = false;

long readUltrasonicCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long dur = pulseIn(ECHO, HIGH, 10000UL);
  if (dur == 0) return 999;
  return (long)(dur / 58);
}

// ================== STATE MACHINE ==================
enum Mode { PRESTART,
            NORMAL,
            AVOID };
Mode mode = PRESTART;

// ================== MOTOR ==================
void tb6612_enable() {
  digitalWrite(STBY, HIGH);
}

void tb6612_disable() {
  digitalWrite(STBY, LOW);
}

void remoteRight(int pwm) {
  tb6612_enable();
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, pwm);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, -pwm);
  }
}

void remoteLeft(int pwm) {
  tb6612_enable();
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, pwm);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -pwm);
  }
}

void stopMotor() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  tb6612_disable();  // tắt motor để tiết kiệm năng lượng
}

// ================== SETUP ==================
void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);  // bật mạch driver

  pinMode(SW, INPUT_PULLUP);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  Serial.begin(9600);
  mode = PRESTART;
}


int getError() {
  int l[9], b[9], count = 0;
  for (int i = 1; i <= 6; i++) {
    l[i] = analogRead(i - 1);
    b[i] = (l[i] > TN) ? 1 : 0;
    count += b[i];
  }
  if(cross_count >= 5 && lost_count >= 3)
  {
    b[2] = 0;
    b[3] = 0;
    count-=2;
  }

 
  // nhận giao nhau: nhiều mắt sáng và đủ 3 vùng (trái/giữa/phải)
  if (count >= 5 && (b[3] || b[4]) && (b[1] || b[2]) && (b[5] || b[6])) {
    cross_count++;
    return 0;
  }
  unsigned long currentMillis = millis();
  bool isLostLine = (count == 0);
  bool isNearCenter = (previous_error_map <= 3 && previous_error_map >= -3);
  int error_to_return = previous_error_map;  // Giá trị lỗi mặc định


  // --- LOGIC 1: ĐANG MẤT LINE (count == 0) ---
  if (isLostLine) {
    found_since = 0;

    if (lost_since == 0) lost_since = currentMillis;

    if (currentMillis - lost_since >= LOST_LINE_THRESHOLD_MS) {
      // mất line đủ lâu
      lost_line_confirmed = true;

      if (lost_count < 3 && isNearCenter) {
        // 3 gap đầu: ép đi thẳng
        return 0;
      } else if (isNearCenter) {
        // sau 3 gap, giữ hướng cũ mạnh hơn 1 chút
        return (previous_error_map > 0) ? +3 : -3;
      } else {
        // đang lệch xa sẵn thì cứ theo lỗi cũ
        return previous_error_map;
      }
    }

    // chưa đủ thời gian để coi là mất line thật
    return previous_error_map;
  }

  // ========== LOGIC KHI CÓ LINE LẠI ==========
  else {
    lost_since = 0;

    if (found_since == 0) found_since = currentMillis;

    if (currentMillis - found_since >= FOUND_LINE_THRESHOLD_MS) {
      if (lost_line_confirmed) {
        if (lost_count < 3 && isNearCenter) {
          lost_count++;  // đếm số gap đã vượt qua
        }
        lost_line_confirmed = false;
      }
    }
  }

  //   if (b[1] && b[2] && b[3] && !b[4] && !b[5] && !b[6]) { // 111000
  //   previous_error_map = -4;    // ép quẹo PHẢI mạnh
  //   return -4;
  // }

  // 011000: S2,S3 dính nhau (gần giống hình nhiễu vuông) -> cũng ép quẹo phải
  // if (!b[1] && b[2] && b[3] && !b[4] && !b[5] && !b[6]) { // 011000
  //   previous_error_map = -4;
  //   return -4;
  // }

  // // 101000: S1,S3 sáng lệch (cục nhiễu + mép line) -> cũng xem là nhiễu
  // if (b[1] && !b[2] && b[3] && !b[4] && !b[5] && !b[6]) { // 101000
  //   previous_error_map = -4;
  //   return -4;
  // }

  // // 110000: chỉ 1-2 sáng (ngoài mép, không có line giữa) -> nhiễu ngoài
  // if (b[1] && b[2] && !b[3] && !b[4] && !b[5] && !b[6]) { // 110000
  //   previous_error_map = -4;
  //   return -4;
  // }

  // // 100000: chỉ S1 sáng -> nhiễu mép ngoài
  // if (b[1] && !b[2] && !b[3] && !b[4] && !b[5] && !b[6]) { // 100000
  //   previous_error_map = -4;
  //   return -4;
  // }

  // // 010000: chỉ S2 sáng, cũng ở mép trái -> đè cho nó quẹo phải luôn
  // if (!b[1] && b[2] && !b[3] && !b[4] && !b[5] && !b[6]) { // 010000
  //   previous_error_map = -3;
  //   return -3;
  // }

  // mapping lỗi (nhẹ)
  if (b[3] && b[4]) {  // 001100
    previous_error_map = 0;
    return 0;
  }

  if (b[3] && b[5]) {  // 001010
    previous_error_map = -4;
    return -4;
  }

  if (b[2] && b[4]) {  // 010100
    previous_error_map = 4;
    return 4;
  }

  if (b[4] && b[6]) {  // 000101
    previous_error_map = -6;
    return -6;
  }

  if (b[1] && b[3]) {  // 101000
    previous_error_map = 6;
    return 6;
  }

  if (b[4] && b[5]) {  // 000110
    previous_error_map = -3;
    return -3;
  }
  if (b[2] && b[3] && b[4]) {  // 011100
    previous_error_map = 5;
    return 5;
  }
  if (b[2] && b[3]) {  // 011000
    previous_error_map = 3;
    return 3;
  }

  if (b[5] && b[6]) {  // 000011
    previous_error_map = -5;
    return -5;
  }

  if (b[1] && b[2]) {  // 110000
    previous_error_map = 5;
    return 5;
  }
  // SINGLE
  if (b[4]) {
    previous_error_map = -2;
    return -2;
  }
  if (b[3]) {
    previous_error_map = 2;
    return 2;
  }
  if (b[5]) {
    previous_error_map = -3;
    return -3;
  }
  if (b[2]) {
    previous_error_map = 3;
    return 3;
  }
  if (b[6]) {
    previous_error_map = -6;
    return -6;
  }
  if (b[1]) {
    previous_error_map = 6;
    return 6;
  }

  return previous_error_map;
}

// ================== PID ==================
int computePID(int err) {
  P = err;
  I += err;
  D = err - previous_error;
  int PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = err;
  return PID_value;
}
void driveLR(int leftPWM, int rightPWM) {
  remoteLeft(leftPWM);
  remoteRight(rightPWM);
}
bool forwardSeekLine(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    int l[9], b[9], count = 0;
    for (int i = 1; i <= 6; i++) {
      l[i] = analogRead(i - 1);
      b[i] = (l[i] > TN) ? 1 : 0;
      count += b[i];
    }
    // chỉ cần bắt lại được line trắng (ít nhất 1 mắt trắng)
    if (count > 0) return true;

    driveLR(BASE, BASE);
    delay(4);
  }
  return false;
}
// ================== LOOP ==================
void loop() {
  switch (mode) {

    // --------- CP1: PRESTART bằng ultrasonic ----------
    case PRESTART:
      {
        stopMotor();

        static unsigned long clear_since = 0;
        long d = readUltrasonicCM();

        if (d <= START_BLOCK_CM) {
          clear_since = 0;  // còn bị chặn -> không chạy
        } else {
          if (clear_since == 0) clear_since = millis();
          if (millis() - clear_since >= START_CLEAR_HOLD_MS) {
            mode = NORMAL;  // xuất phát
            startflag_done = true;
          }
        }
        break;
      }

    // --------- CP2: NORMAL ----------
    case NORMAL:
      {
        unsigned long now = millis();

        // ==== CHECK VẬT CẢN CHỈ SAU KHI ĐÃ QUA CỜ XUẤT PHÁT ====
        if ((startflag_done || lost_count >= 3 ||cross_count >= 5) && now - lastUltraCheck >= ULTRA_PERIOD_MS) {
          lastUltraCheck = now;
          long d = readUltrasonicCM();

          if (d <= OBSTACLE_THRESHOLD_CM) {
            if (block_since == 0) block_since = now;
            if (now - block_since >= OBST_BLOCK_HOLD_MS) {
              obstacle_confirmed = true;
            }
          } else {
            block_since = 0;
          }

          if (obstacle_confirmed && !avoid_done) {
            // chuyển sang mode né vật cản
            stopMotor();
            obstacle_confirmed = false;
            block_since = 0;
            mode = AVOID;
            break;  // ra khỏi case NORMAL, vòng lặp sau sẽ vào case AVOID
          }
        }

        // ===== PID bình thường khi không có vật cản =====
        error = getError();
        PIDValue = computePID(error);

        int pwmL = BASE - PIDValue;
        int pwmR = BASE + PIDValue;
        pwmL = constrain(pwmL, 0, PWM_MAX);
        pwmR = constrain(pwmR, 0, PWM_MAX);
        remoteLeft(pwmL);
        remoteRight(pwmR);

        break;
      }

    // --------- MODE AVOID: SCRIPT DỪNG + RẼ PHẢI + QUẸO PHẢI VÔ LẠI + BẮT LINE ----------
    case AVOID:
      {
        // 1) dừng nhẹ
        stopMotor();
        delay(500);

        // 2) rẽ phải ra khỏi line (xoay phải tại chỗ)
        driveLR(BASE, -BASE);  // trái tiến, phải lùi => quay phải
        delay(280);

        // 3) đi thẳng 1 đoạn để vượt khỏi vật cản
        driveLR(BASE, BASE);
        delay(520);

        // 4) quẹo trái thêm lần nữa để hướng lại về đường cũ (tùy góc m muốn)
        driveLR(-BASE, BASE);
        delay(520);

        // 5) chạy thẳng tìm lại line trong tối đa 2s
        if(forwardSeekLine(1000))
        {
          stopMotor();
          delay(35);

        // Sau khi bắt line (hoặc hết timeout), quay lại mode NORMAL
          mode = NORMAL;
          avoid_done = true;
        }

        break;
      }
  }
}
