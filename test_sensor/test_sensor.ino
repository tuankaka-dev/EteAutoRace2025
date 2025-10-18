#define ENA 5
#define ENB 6

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
// Tốc độ
const int BASE = 100;  // speed (0..255)
const int PWM_MAX = 255;

//  PID
const int Kp = 8, Ki = 0, Kd = 5;
int P, I = 0, D;
int previous_error = 0;
int last_seen_error = 0;
int error, PIDValue;

// pwm: -255..255 (âm = lùi)
void remoteLeft(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwm);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -pwm);
  }
}

void remoteRight(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwm);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -pwm);
  }
}




void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

void loop() {


  error = getError();

  PIDValue = computePID(error);
  PIDValue = constrain(PIDValue, -70, 70);
  // Trộn BASE ± PID -> PWM cho 2 bánh
  //    Quy ước: error dương (lệch phải) => tăng trái, giảm phải
  int pwmL = BASE + PIDValue;
  int pwmR = BASE - PIDValue;

  pwmL = constrain(pwmL, 0, PWM_MAX);
  pwmR = constrain(pwmR, 0, PWM_MAX);

  remoteLeft(pwmL);
  remoteRight(pwmR);

    // ==== GỌI TEST SENSOR & IN DEBUG MỖI 100ms ====
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    readIRSensors();                         // cập nhật irRaw[] và irBits
    printLineDebug(error, PIDValue, pwmL, pwmR);
    lastPrint = millis();
  }
}
// ================== DEBUG SENSOR BLOCK (ADD) ==================
const int IR_PINS[5] = {A1, A2, A3, A4, A5};   // trùng thứ tự trong getError()
const int IR_W[5]   = {-2, -1, 0, 1, 2};       // tham khảo (nếu cần)
const int THR = 500;                           // nhớ giữ đồng bộ với getError()

int irRaw[5];           // giá trị analog từng kênh
uint8_t irBits = 0;     // bitmask trạng thái 5 kênh (A1..A5 -> bit4..bit0)

// Đọc toàn bộ 5 cảm biến, lấp đầy irRaw[] và irBits
void readIRSensors() {
  irBits = 0;
  for (int i = 0; i < 5; i++) {
    irRaw[i] = analogRead(IR_PINS[i]);
    if (irRaw[i] < THR) {  /// line đen < line trắng >
      // A1 là trái nhất -> map vào bit cao nhất (bit4)
      irBits |= (1 << (4 - i));
    }
  }
}

// In một dòng debug: RAW, DIG, err, P/I/D, PID, PWM L/R
void printLineDebug(int err, int pid, int pwmL, int pwmR) {
  Serial.print(F("RAW:"));
  for (int i = 0; i < 5; i++) {
    Serial.print(' ');
    Serial.print(irRaw[i]);
  }

  Serial.print(F(" | DIG: "));
  for (int i = 0; i < 5; i++) {
    // In theo thứ tự A1..A5
    Serial.print( ((irBits >> (4 - i)) & 1) ? '1' : '0' );
  }

  Serial.print(F(" | err="));   Serial.print(err);
  Serial.print(F(" | P/I/D=")); Serial.print(P); Serial.print('/'); Serial.print(I); Serial.print('/'); Serial.print(D);
  Serial.print(F(" | PID="));   Serial.print(pid);
  Serial.print(F(" | PWM L/R=")); Serial.print(pwmL); Serial.print('/'); Serial.println(pwmR);
}
// ================== END DEBUG SENSOR BLOCK ==================

int getError() {
  // Thứ tự cảm biến: A1 trái nhất -> A5 phải nhất
  const int S[5] = {A1, A2, A3, A4, A5};
  const int W[5] = {-2, -1, 0, 1, 2};   // Trái âm, phải dương
  const int THRESH = 400;               // line trắng > 500, nền đen < 500

  int count = 0;
  long sum = 0;

  for (int i = 0; i < 5; i++) {
    int val = analogRead(S[i]);
    bool onLine = (val < THRESH); // trắng >
    if (onLine) {
      sum += W[i];
      count++;
    }
  }

  // Mất line -> xoay nhẹ theo hướng trước đó
  if (count == 0) {
    if (last_seen_error > 0) return 3;
    if (last_seen_error < 0) return -3;
    return 0;
  }

  int err = (int)(sum / count);
  last_seen_error = err;
  return err;
}



int computePID(int error) {
  P = error;                   // Proportional

  if (Ki != 0) {                 // tránh tích lũy khi Ki = 0
    I += error; //
    // I = constrain(I, -50, 50); // nếu sau này dùng Ki>0 thì mở dòng này
  } else {
    I = 0;
  }               

  D = error - previous_error;  // Differential

  int PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;
  return PID_value;
}