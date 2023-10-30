// Khai báo các chân điều khiển motor
#define in1 7
#define in2 6
#define in3 5
#define in4 4
#define MAX_SPEED 255
#define MIN_SPEED 0

// Khai báo các chân module HC SRO5
const int U4_trig = 22; // Cảm biến KC bên trái
const int U4_echo = 23;
const int U3_trig = 24; // Cảm biến KC trước trái
const int U3_echo = 25;
const int U2_trig = 26; // Cảm biến KC trước
const int U2_echo = 27;
const int U1_trig = 28; // Cảm biến KC trước phải
const int U1_echo = 29;
const int U0_trig = 30; // Cảm biến KC bên phải
const int U0_echo = 31;

void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //
  pinMode(U4_trig,OUTPUT); // Chân trig phát tín hiệu
  pinMode(U4_echo,INPUT); // Chân echo nhận tín hiệu
  pinMode(U3_trig,OUTPUT);
  pinMode(U3_echo,INPUT);
  pinMode(U2_trig,OUTPUT);
  pinMode(U2_echo,INPUT);
  pinMode(U1_trig,OUTPUT);
  pinMode(U1_echo,INPUT);
  pinMode(U0_trig,OUTPUT);
  pinMode(U0_echo,INPUT);
}

void loop() {
  int sum1 = caculate_Distance(U4_trig, U4_echo);
  if (sum1 >= 30) {sum1 = 1;}
  else {sum1 = 0;}

  int sum2 = caculate_Distance(U3_trig, U3_echo);
  if (sum2 >= 30) {sum2 = 1;}
  else {sum2 = 0;}

  int sum3 = caculate_Distance(U2_trig, U2_echo);
  if (sum3 >= 30) {sum3 = 1;}
  else {sum3 = 0;}

  int sum4 = caculate_Distance(U1_trig, U1_echo);
  if (sum4 >= 30) {sum4 = 1;}
  else {sum4 = 0;}

  int sum5 = caculate_Distance(U0_trig, U0_echo);
  if (sum5 >= 30) {sum5 = 1;}
  else {sum5 = 0;}
  
  int sum = sum1 + sum2 + sum3 + sum4 + sum5;
  if(sum  == 1 || sum == 3 || sum == 5) {
    motor1_Move(150);
    motor2_Move(50);
  }  
  else {
    motor1_Move(255);
    motor2_Move(255);
  }
}

void motor1_Move(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  analogWrite(in1, speed);
  digitalWrite(in2, LOW);
}

void motor2_Move(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  analogWrite(in3, speed);
  digitalWrite(in4, LOW);
}

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

int caculate_Distance(int trig, int echo) {
  unsigned long duration;
  int distance;

  /* Phát xung từ chân trig */
  digitalWrite(trig,0);   // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(trig,1);   // phát xung từ chân trig
  delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  digitalWrite(trig,0);   // tắt chân trig

  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo. 
  duration = pulseIn(echo,HIGH);  
  // Tính khoảng cách đến vật.
  distance = int(duration/2/29.412);
  return distance;
}
