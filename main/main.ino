#define ENA 5
#define ENB 6

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
// Tốc độ
const int BASE = 110;  // speed (90..255)
const int PWM_MAX = 255;

//  PID
const int Kp = 20, Ki = 0, Kd = 7;
int P, I = 0, D;
int previous_error_map = 0;
int previous_error = 0;
int last_seen_error = 0;
int error, PIDValue;

int e[32];

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
  initErrMap5(); 
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
void initErrMap5() {
    for (int i = 0; i < 32; ++i) e[i] = 0; 

    // đơn điểm
    e[0b10000] = -4; // 16
    e[0b01000] = -2; // 8
    e[0b00100] =  0; // 4 (giữa)
    e[0b00010] = +2; // 2
    e[0b00001] = +4; // 1

    // đôi kề nhau
    e[0b11000] = -4; // 24
    e[0b01100] = -1; // 12
    e[0b00110] = +1; // 6
    e[0b00011] = +4; // 3

    // cụm rộng (3 cảm biến)
    e[0b11100] = -3; // 28
    e[0b01110] =  0; // 14 (rộng ở giữa)
    e[0b00111] = +3; // 7

    // mép/ngoại lệ (hai mép cùng sáng)
    e[0b10001] = -4; // 17 (sẽ đảo dấu theo previous_error ở dưới)
}
int getError() {
    int l[6], b[6];


    l[1] = analogRead(1);
    l[2] = analogRead(2);
    l[3] = analogRead(3);
    l[4] = analogRead(4);
    l[5] = analogRead(5);

    //  line trắng ~815, đen ~12)
    b[1] = (l[1] < 555) ? 1 : 0; // trái
    b[2] = (l[2] < 505) ? 1 : 0;
    b[3] = (l[3] < 500) ? 1 : 0; // giữa
    b[4] = (l[4] < 380) ? 1 : 0;
    b[5] = (l[5] < 485) ? 1 : 0; // phải

    error = findError5(b[1], b[2], b[3], b[4], b[5]);
    return error;

}
int findError5(int s1, int s2, int s3, int s4, int s5) {
    int val = s5 + 2*s4 + 4*s3 + 8*s2 + 16*s1;
    int res = e[val];


    if (val == 14 || val == 17) {
        if (previous_error_map > 0) res = -res;
    }
    else if (val == 0) {
      return (previous_error_map >= 0) ? +3 : -3;  
}

    previous_error_map = res;
    return res;
}


int computePID(int error) {
  P = error;                   // Proportional
  I += error; //
  D = error - previous_error;  // Differential

  int PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;
  return PID_value;
}