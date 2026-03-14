// ========= MOTOR PINS =========
#define PWMA 25
#define PWMB 26
#define AIN1 17
#define AIN2 16
#define BIN1 27
#define BIN2 13
#define STBY 21

// ========= PWM SETTINGS =========
#define freq 1000
#define resolution 8

#define channelA 0
#define channelB 1

int speedMotor = 180; // 0-255

void setup() {

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH); // Wake driver

  // PWM setup
  ledcSetup(channelA, freq, resolution);
  ledcAttachPin(PWMA, channelA);

  ledcSetup(channelB, freq, resolution);
  ledcAttachPin(PWMB, channelB);
}

// ================= MOTOR FUNCTIONS =================

void forward() {

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  ledcWrite(channelA, speedMotor);
  ledcWrite(channelB, speedMotor);
}

void backward() {

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  ledcWrite(channelA, speedMotor);
  ledcWrite(channelB, speedMotor);
}

void left() {

  ledcWrite(channelA, 80);   // slow left motor
  ledcWrite(channelB, 200);  // fast right motor

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void right() {

  ledcWrite(channelA, 200);
  ledcWrite(channelB, 80);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void stopMotor() {

  ledcWrite(channelA, 0);
  ledcWrite(channelB, 0);
}

// ================= TEST LOOP =================

void loop() {

  forward();
  delay(3000);

  stopMotor();
  delay(1000);

  backward();
  delay(3000);

  stopMotor();
  delay(1000);

  left();
  delay(2000);

  right();
  delay(2000);

  stopMotor();
  delay(4000);
}
