#define ENA 5
#define ENB 6

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
// Tốc độ
const int BASE = 130;  // speed (90..255)
const int PWM_MAX = 255;

//  PID
const int Kp = 18, Ki = 0, Kd = 1;
int P, I = 0, D;
int previous_error = 0;
int last_seen_error = 0;
int error, PIDValue;

// pwm: -255..255 (âm = lùi)
void remoteRight(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwm);
  }
}

void remoteLeft(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
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
  // PIDValue = constrain(PIDValue, -80, 80);
  // Trộn BASE ± PID -> PWM cho 2 bánh
  //    Quy ước: error dương (lệch phải) => tăng trái, giảm phải
  int pwmL = BASE + PIDValue;
  int pwmR = BASE - PIDValue;

  pwmL = constrain(pwmL, 0, PWM_MAX);
  pwmR = constrain(pwmR, 0, PWM_MAX);

  remoteLeft(pwmL);
  remoteRight(pwmR);


}

int getError() {
  // Thứ tự cảm biến: A1 trái nhất -> A5 phải nhất
  const int S[5] = {A1, A2, A3, A4, A5};
  const int W[5] = {-3, -1, 0, 1, 3};   // Trái âm, phải dương
  const int THRESH = 400;               // line trắng > 500, nền đen < 500

  int count = 0;
  long sum = 0;

  for (int i = 0; i < 5; i++) {
    int val = analogRead(S[i]);
    bool onLine = (val < THRESH); // trắng (line) → true
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
  I += error; //
  D = error - previous_error;  // Differential

  int PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;
  return PID_value;
}