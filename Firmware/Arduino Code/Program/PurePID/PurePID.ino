// ========================================
// LINE FOLLOWER v6.2 — PURE PID, NO ADAPTIVE SPEED
// ESP32 + TB6612FNG + 8 IR Sensors
// ========================================

#define AIN1 17
#define AIN2 16
#define BIN1 27
#define BIN2 13
#define PWMA 25
#define PWMB 26
#define STBY 21

#define IR1 23
#define IR8 22
int IR_analog[6] = {36, 39, 34, 35, 32, 33};

const int THRESHOLD = 600;

float Kp = 50.0f;
float Ki = 0.0f;
float Kd = 8.5f;

float integral      = 0;
float lastError     = 0;
float filteredError = 0;

const float sensorPos[6] = {-3.75f, -2.25f, -0.75f, 0.75f, 2.25f, 3.75f};

int baseSpeed     = 250;
int maxSpeed      = 255;

int currentSpeed = 0;
const int rampStep  = 10;
const int rampDelay = 10;
unsigned long lastRampTime = 0;

const float DEADBAND = 0.08f;

unsigned long lastLineSeenTime = 0;

const unsigned long DASHED_GAP_TIMEOUT = 400;
const unsigned long SPIN_TIMEOUT       = 800;
const unsigned long REVERSE_TIMEOUT    = 500;
const unsigned long SPIN2_TIMEOUT      = 800;

enum RecoveryPhase {
  ON_LINE,
  DASHED_GAP,
  SPINNING,
  REVERSING,
  SPINNING2,
  GAVE_UP
};

RecoveryPhase recoveryPhase    = ON_LINE;
unsigned long recoveryPhaseStart = 0;
int spinDirection = 0;

int rawADC[6];
int IR_inner[6];
int IR_left  = 0;
int IR_right = 0;

unsigned long lastLoopTime = 0;
float dt = 0.005f;

bool running = false;

// ============================================================
void setup(){
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(IR1, INPUT);
  pinMode(IR8, INPUT);
  for(int i = 0; i < 6; i++) pinMode(IR_analog[i], INPUT);

  ledcSetup(0, 2000, 8); ledcAttachPin(PWMA, 0);
  ledcSetup(1, 2000, 8); ledcAttachPin(PWMB, 1);

  stopMotors();

  running            = true;
  currentSpeed       = 0;
  lastRampTime       = millis();
  lastLineSeenTime   = millis();
  lastLoopTime       = micros();
  integral           = 0;
  lastError          = 0;
  filteredError      = 0;
  recoveryPhase      = ON_LINE;
}

// ============================================================
void drive(int L, int R){
  L = constrain(L, -maxSpeed, maxSpeed);
  R = constrain(R, -maxSpeed, maxSpeed);

  if(L >= 0){ digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else       { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); L = -L; }
  ledcWrite(0, L);

  if(R >= 0){ digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else       { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); R = -R; }
  ledcWrite(1, R);
}

void stopMotors(){
  ledcWrite(0, 0); ledcWrite(1, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

// ============================================================
void readSensors(){
  IR_left  = (digitalRead(IR1) == HIGH) ? 1 : 0;
  IR_right = (digitalRead(IR8) == HIGH) ? 1 : 0;

  for(int i = 0; i < 6; i++){
    rawADC[i]   = analogRead(IR_analog[i]);
    IR_inner[i] = (rawADC[i] > THRESHOLD) ? 1 : 0;
  }
}

// ============================================================
float computeError(){
  float weightedSum = 0;
  float totalWeight = 0;

  for(int i = 0; i < 6; i++){
    float strength = (float)rawADC[i] - (float)THRESHOLD;
    if(strength < 0) strength = 0;
    weightedSum += sensorPos[i] * strength;
    totalWeight += strength;
  }

  if(totalWeight < 100) return lastError;

  lastLineSeenTime = millis();
  return weightedSum / totalWeight;
}

// ============================================================
bool anyInnerOnLine(){
  for(int i = 0; i < 6; i++) if(IR_inner[i]) return true;
  return false;
}

// ============================================================
void handleSpeedRamp(){
  if(millis() - lastRampTime >= rampDelay){
    lastRampTime = millis();
    if(currentSpeed < baseSpeed) currentSpeed += rampStep;
  }
}

// ============================================================
bool handleRecovery(){

  bool innerSeen = anyInnerOnLine();

  if(innerSeen){
    if(recoveryPhase != ON_LINE){
      integral      = 0;
      filteredError = 0;
      recoveryPhase = ON_LINE;
    }
    lastLineSeenTime = millis();
    return false;
  }

  if(recoveryPhase == ON_LINE){
    spinDirection      = (lastError >= 0) ? 1 : -1;
    recoveryPhase      = DASHED_GAP;
    recoveryPhaseStart = millis();
  }

  unsigned long phaseElapsed = millis() - recoveryPhaseStart;

  switch(recoveryPhase){

    case DASHED_GAP:
      {
        float correction = lastError * Kp * 0.1f;
        int L = constrain(currentSpeed + (int)correction, 0, maxSpeed);
        int R = constrain(currentSpeed - (int)correction, 0, maxSpeed);
        drive(L, R);

        if(phaseElapsed >= DASHED_GAP_TIMEOUT){
          recoveryPhase      = SPINNING;
          recoveryPhaseStart = millis();
        }
      }
      break;

    case SPINNING:
      if(spinDirection > 0) drive( 180, -180);
      else                  drive(-180,  180);

      if(phaseElapsed >= SPIN_TIMEOUT){
        recoveryPhase      = REVERSING;
        recoveryPhaseStart = millis();
      }
      break;

    case REVERSING:
      drive(-120, -120);

      if(phaseElapsed >= REVERSE_TIMEOUT){
        spinDirection      = -spinDirection;
        recoveryPhase      = SPINNING2;
        recoveryPhaseStart = millis();
      }
      break;

    case SPINNING2:
      if(spinDirection > 0) drive( 180, -180);
      else                  drive(-180,  180);

      if(phaseElapsed >= SPIN2_TIMEOUT){
        recoveryPhase = GAVE_UP;
        stopMotors();
        running = false;
      }
      break;

    case GAVE_UP:
      stopMotors();
      running = false;
      break;

    default:
      break;
  }

  return true;
}

// ============================================================
void pidControl(){

  readSensors();

  unsigned long now = micros();
  dt = (now - lastLoopTime) / 1000000.0f;
  dt = constrain(dt, 0.001f, 0.05f);
  lastLoopTime = now;

  if(handleRecovery()) return;

  float error = computeError();

  if(fabsf(error) < DEADBAND) error = 0.0f;

  filteredError = 0.5f * filteredError + 0.5f * error;

  float P_term = Kp * filteredError;
  float D_term = Kd * (filteredError - lastError);

  if(anyInnerOnLine())
    integral += filteredError * dt;
  else
    integral = 0;
  integral = constrain(integral, -30.0f, 30.0f);
  float I_term = Ki * integral;

  float correction = P_term + I_term + D_term;

  lastError = filteredError;

  // Pure PID — currentSpeed is the only speed source
  // No adaptive reduction — bot runs at full ramp speed always
  int L = currentSpeed + (int)correction;
  int R = currentSpeed - (int)correction;

  int maxVal = max(abs(L), abs(R));
  if(maxVal > maxSpeed){
    L = (L * maxSpeed) / maxVal;
    R = (R * maxSpeed) / maxVal;
  }

  drive(L, R);
}

// ============================================================
void loop(){
  if(running){
    handleSpeedRamp();
    pidControl();
  } else {
    stopMotors();
  }
}