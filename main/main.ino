#define ENA 5
#define ENB 6

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
// Tốc độ 
const int BASE = 80;      // speed (0..255)
const int PWM_MAX = 255;

//  PID 
const int Kp = 2, Ki = 0, Kd = 3 ;
int P, I = 0, D, previous_error = 0;

int error, PIDValue;

// pwm: -255..255 (âm = lùi)
void motorLeft(int pwm){
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0){ digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  analogWrite(ENA, pwm); }
  else         { digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW); analogWrite(ENA, -pwm); }
}
void motorRight(int pwm){
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0){ digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  analogWrite(ENB, pwm); }
  else         { digitalWrite(IN3, HIGH);  digitalWrite(IN4, LOW); analogWrite(ENB, -pwm); }
}

void remoteLeft(int pwm){ motorLeft(pwm); }
void remoteRight(int pwm){ motorRight(pwm); }


void setup()
{
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  Serial.begin(9600);
}

void loop()
{


  error = getError();

  PIDValue = computePID(error);

  // Trộn BASE ± PID -> PWM cho 2 bánh
  //    Quy ước: error dương (lệch phải) => tăng trái, giảm phải
  int pwmL = BASE + PIDValue;
  int pwmR = BASE - PIDValue;


  pwmL = constrain(pwmL, 0, PWM_MAX);
  pwmR = constrain(pwmR, 0, PWM_MAX);


  

  remoteLeft(pwmL);
  remoteRight(pwmR);
  
  delay(100);
  
}

int getError()
{
    int l[6], b[6];
    l[1] = analogRead(A1);
    l[2] = analogRead(A2);
    l[3] = analogRead(A3);
    l[4] = analogRead(A4);
    l[5] = analogRead(A5);
    // 1 là đen 0 là trắng (giá trị sensor đen:8-15 ; trắng 800 - 820)
    if (l[1] < 500) b[1] = 1; else b[1] = 0;
    if (l[2] < 500) b[2] = 1; else b[2] = 0;
    if (l[3] < 500) b[3] = 1; else b[3] = 0;
    if (l[4] < 500) b[4] = 1; else b[4] = 0;
    if (l[5] < 500) b[5] = 1; else b[5] = 0;



    if (b[3] == 0 && b[4] == 0)
    return 0;

    // Nhận 3 cảm biến (nhiễu): coi như lệch nhẹ
    else if (b[2] == 0 && b[3] == 0 && b[4] == 0)
        return 0;
    else if (b[3] == 0 && b[4] == 0 && b[5] == 0)
        return 1;
    else if (b[1] == 0 && b[2] == 0 && b[3] == 0)
        return -1;

    // Nhận 2 cảm biến liền kề
    else if (b[4] == 0 && b[5] == 0)
        return 2;
    else if (b[1] == 0 && b[2] == 0)
        return -2;
    else if (b[3] == 0 && b[5] == 0)
        return 1;
    else if (b[2] == 0 && b[4] == 0)
        return -1;

    // Nhận 1 cảm biến
    else if (b[3] == 0) // giữa
        return 0;
    else if (b[4] == 0)
        return 1;
    else if (b[2] == 0)
        return -1;
    else if (b[5] == 0)
        return 3;
    else if (b[1] == 0)
        return -3;
}

int computePID(int error)
{
    P = error;                  // Proportional
    I += error;                 // Integral
    D = error - previous_error; // Differential

    int PID_value = (Kp * P) + (Ki * I) + (Kd * D);

    previous_error = error;
    return PID_value;
}
