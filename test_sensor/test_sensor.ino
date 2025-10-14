  // === TEST HAM getError() CHO 5 TCRT5000 ===
  // Nền đen – line trắng  →  line trắng = 0, nền đen = 1
  // A1..A5 nối lần lượt với 5 cảm biến từ trái sang phải

  void setup() {
    Serial.begin(9600);
    delay(1000);
    Serial.println("=== BAT DAU TEST HAM getError() ===");
  }


  // =================== getError() ===================
  // Dùng biến toàn cục để in ra ngoài loop
  int l[6], b[6];
  const int BASE = 80;      // speed (0..255)
const int PWM_MAX = 255;
  const int Kp = 25, Ki = 0, Kd = 8 ;
int P, I = 0, D, previous_error = 0;
int error, PIDValue;
  int getError()
  {

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
  
  void loop() {
    int error = getError();   // gọi hàm
  PIDValue = computePID(error);

  // Trộn BASE ± PID -> PWM cho 2 bánh
  //    Quy ước: error dương (lệch phải) => tăng trái, giảm phải
  int pwmL = BASE + PIDValue;
  int pwmR = BASE - PIDValue;


  pwmL = constrain(pwmL, 0, PWM_MAX);
  pwmR = constrain(pwmR, 0, PWM_MAX);

    Serial.print("Error = ");
    Serial.print(error);
   
    // // In ra mảng b[1..5] để xem cảm biến nhận gì
     //Serial.print(" | Binary: ");
    // for (int i = 1; i <= 5; i++) {
    //   Serial.print(b[i]);
    //   Serial.print(" ");
    // }

    // Serial.print(" | Analog: ");
    // for (int i = 1; i <= 5; i++) {
    //   Serial.print(l[i]);
    //   Serial.print("\t");
    // }
    // Serial.println();
        // === DEBUG: in giá trị ra Serial Monitor ===


Serial.print(" P=");  Serial.print(P);
Serial.print(" I=");  Serial.print(I);
Serial.print(" D=");  Serial.print(D);
Serial.print(" | PID="); Serial.print(PIDValue);
Serial.print(" | pwmL="); Serial.print(pwmL);
Serial.print(" pwmR=");  Serial.print(pwmR);
Serial.println();

    delay(300); // đợi 200ms cho dễ đọc
  }
