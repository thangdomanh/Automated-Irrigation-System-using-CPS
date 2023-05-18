#define IN1        11
#define IN2        10
#define IN3        9
#define IN4        8
#define MAX_SPEED  255  // từ 0-255
#define MIN_SPEED  0
#define SENSOR_PIN A0
#define SETPOINT 65.0

double input, output, check;
double kp, ki, kd;
double lastInput, errorSum;
unsigned long lastTime;
double elapsedTime;

void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Khởi tạo các biến và hệ số
  input = 0;
  output = 0;
  kp = 0;
  ki = 0;
  kd = 0;
  lastInput = 0;
  errorSum = 0;
  lastTime = millis();
  check=0;

  // Tính toán hệ số
  computePIDCoefficients();
}
void motor_1_Dung()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void quat_Dung()
{
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void motor_1_Tien(int speed)  // speed: từ 0 - MAX_SPEED
{
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(IN1, HIGH);
  analogWrite(IN2, 255 - speed);
}

void quat_ChayThuan(int speed)  // speed: từ 0 - MAX_SPEED
{
  speed = constrain(speed+155, MIN_SPEED, MAX_SPEED);
  analogWrite(IN3, speed);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Đọc giá trị độ ẩm từ cảm biến
  input = readMoistureLevel();

  // Tính toán giá trị PID
  computePIDOutput();

  // In giá trị độ ẩm và điều khiển PID
  Serial.print("Độ ẩm: ");
  Serial.print(input);
  Serial.print("%");
  Serial.print(" - Điều khiển: ");
  Serial.println(output);

  // Đợi 1 giây
  delay(1000);
}

double readMoistureLevel() {
  int sensorValue = analogRead(SENSOR_PIN);
  double moisture = map(sensorValue, 1023, 0, 0, 100);
  return moisture;
}

void computePIDOutput() {
  unsigned long currentTime = millis();
  elapsedTime = (double)(currentTime - lastTime) / 1000.0;

  // Tính toán lỗi hiện tại
  double error = SETPOINT - input;
  Serial.print(errorSum);

//Kiem tra dieu khien quat
  if(error < 0){
    check = 1;
    error = abs(error);
  } else if (error > 0)
  {
    check = 0;
  }

  // Tính toán thành phần P
  double p = kp * error;

  // Tính toán thành phần I
  errorSum += error * elapsedTime;
  double i = ki * errorSum;
  
  // Tính toán thành phần D
  double d = kd * (input - lastInput) / elapsedTime;

  // Tổng hợp các thành phần để tính toán đầu ra PID
  output = p + i + d;
  // Giới hạn đầu ra trong khoảng từ 0 đến 100
  output = constrain(output, 0, 100);

  //Sai so cho phep so voi Setpoint
  if (error < 0.046*SETPOINT){
    errorSum = 0;
    output=0;
  }

if (check == 1 && output > 0)
  {
    motor_1_Dung();  // Dừng motor 1
    quat_ChayThuan(output);  // Quạt chạy thuận
  }
  else if (check == 0 && output > 0)
  {
    motor_1_Tien(output);  // Motor 1 tiến
    quat_Dung();  // Dừng quạt
  }
  else
  {
    motor_1_Dung();  // Dừng motor 1
    quat_Dung();  // Dừng quạt
  }
  // Lưu giá trị đầu vào và thời gian hiện tại
  lastInput = input;
  lastTime = currentTime;
}

void computePIDCoefficients() {
  // Tính toán hệ số Kp, Ki và Kd theo phương pháp thử và sai
  kp = 0.7;  // Hệ số tỷ lệ
  ki = 0.7;  // Hệ số tích phân
  kd = 1;  // Hệ số vi phân
}
