const int ENA = 5;      
const int ENB = 6;      

const int IN1 = 8; // Motor trái
const int IN2 = 9;
const int IN3 = 10; // Motor phải
const int IN4 = 11;

const int TRIG = 12; // Cảm biến siêu âm
const int ECHO = 13;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  Serial.begin(115200);
}
// Motor trái
void motorLeft(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
  analogWrite(ENA, speed);
}

// Motor phải
void motorRight(int speed, bool forward) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, HIGH); // Dùng digitalWrite(HIGH) thay cho analogWrite(speed)
}
// Xe tiến
void forward(int speed) {
  motorLeft(speed, true);
  motorRight(speed, true);
}

// Xe lùi
void backward(int speed) {
  motorLeft(speed, false);
  motorRight(speed, false);
}

// Quay trái tại chỗ
void spinLeft(int speed) {
  motorLeft(speed, false);
  motorRight(speed, true);
}

// Quay phải tại chỗ
void spinRight(int speed) {
  motorLeft(speed, true);
  motorRight(speed, false);
}

// Dừng xe
void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {  
  forward(150);   // chạy thẳng
  delay(2000);

  stopMotor();
  delay(500);




 
  // backward(150);  // lùi
  // delay(2000);

  // stopMotor();

  // delay(1000);  
}
