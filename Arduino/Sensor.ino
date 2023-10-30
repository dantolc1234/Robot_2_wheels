#define in1 7
#define in2 6
#define in3 5
#define in4 4
#define MAX_SPEED 255
#define MIN_SPEED 0

const int U2_trig = 26; // Cảm biến KC trước
const int U2_echo = 27;

void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(U2_trig,OUTPUT);
  pinMode(U2_echo,INPUT);
}

void loop() {
  unsigned long duration;
  int distance;

  /* Phát xung từ chân trig */
  digitalWrite(U2_trig,0);   // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(U2_trig,1);   // phát xung từ chân trig
  delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  digitalWrite(U2_trig,0);   // tắt chân trig

  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo. 
  duration = pulseIn(U2_echo,HIGH);  
  // Tính khoảng cách đến vật.
  distance = int(duration/2/29.412);
  int speed = 50;
  int roll = 250;
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  if (distance < 50)
  {
    digitalWrite(in1, LOW);
    analogWrite(in2, roll);
    digitalWrite(in3, LOW);
    analogWrite(in4, roll);
  }
  else
  {
    digitalWrite(in1, LOW);
    analogWrite(in2, speed);
    analogWrite(in3, speed);
    digitalWrite(in4, LOW);
  }
}
