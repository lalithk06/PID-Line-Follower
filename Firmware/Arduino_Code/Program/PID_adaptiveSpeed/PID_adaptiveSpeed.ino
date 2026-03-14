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

const int THRESHOLD = 100;

float Kp = 39.5f;
float Ki = 0.0f;
float Kd = 8.8f;

float integral      = 0;
float lastError     = 0;
float filteredError = 0;

const float sensorPos[6] = {-3.5f, -2.0f, -0.5f, 0.5f, 2.0f, 3.5f};

int   baseSpeed      = 115;   // ← raise to 255 if motors handle it
int   maxSpeed       = 255;
int   MIN_CURVE_SPEED = 60;  // ← floor during sharp curves

const float SPEED_KP = 16.0f;

int currentSpeed = 0;
const int rampStep  = 10;    // faster ramp — reaches full speed quicker
const int rampDelay = 10;    // ms between ramp steps
unsigned long lastRampTime = 0;

const float DEADBAND = 0.08f;

// ================= RECOVERY =================
unsigned long lastLineSeenTime = 0;
const unsigned long DASHED_GAP_TIMEOUT = 400;
const unsigned long SPIN_TIMEOUT       = 500;

const int REVERSE_SPEED = 70;

enum RecoveryPhase {
  ON_LINE,
  DASHED_GAP,
  SPINNING,
  REVERSING,
  SPINNING2,
};

RecoveryPhase recoveryPhase    = ON_LINE;
unsigned long recoveryPhaseStart = 0;
int spinDirection = 0;

// ================= SENSOR DATA =================
int rawADC[6];
int IR_inner[6];
int IR_left  = 0;
int IR_right = 0;

unsigned long lastLoopTime = 0;
float dt = 0.005f;

bool running = false;

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

void readSensors(){
  IR_left  = (digitalRead(IR1) == HIGH) ? 1 : 0;
  IR_right = (digitalRead(IR8) == HIGH) ? 1 : 0;

  for(int i = 0; i < 6; i++){
    rawADC[i]   = analogRead(IR_analog[i]);
    IR_inner[i] = (rawADC[i] > THRESHOLD) ? 1 : 0;
  }
}

float computeError(){
  float weightedSum = 0;
  float totalWeight = 0;

  for(int i = 0; i < 6; i++){
    float strength = (float)rawADC[i] - (float)THRESHOLD;
    if(strength < 0) strength = 0;
    weightedSum += sensorPos[i] * strength;
    totalWeight += strength;
  }

  if(totalWeight < 10) return lastError;

  lastLineSeenTime = millis();
  return weightedSum / totalWeight;
}

bool anyInnerOnLine(){
  for(int i = 0; i < 6; i++) if(IR_inner[i]) return true;
  return false;
}

void handleSpeedRamp(){
  if(millis() - lastRampTime >= rampDelay){
    lastRampTime = millis();
    if(currentSpeed < baseSpeed) currentSpeed += rampStep;
  }
}


bool handleRecovery(){

  bool innerSeen = anyInnerOnLine();

  if(innerSeen){
    if(recoveryPhase != ON_LINE){
      integral      = 0;
      filteredError = 0;
      lastError     = 0;
      recoveryPhase = ON_LINE;
    }
    lastLineSeenTime = millis();
    return false;
  }

  // Line is lost — start or continue recovery
  if(recoveryPhase == ON_LINE){
    spinDirection      = (lastError >= 0) ? 1 : -1;
    recoveryPhase      = SPINNING;
    recoveryPhaseStart = millis();
}

  unsigned long phaseElapsed = millis() - recoveryPhaseStart;

  switch(recoveryPhase){

    case DASHED_GAP:
      {
        // Brief forward coast to handle dashed gaps
        float correction = lastError * Kp * 0.3f;
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
      if(phaseElapsed < 100){
        stopMotors();          // brief stop before spin
      } else {
        if(spinDirection > 0) drive( 130, -130);
        else                  drive(-130,  130);
      }

      break;

    default:
      break;
  }

  return true;
}

enum TurnState { TURN_NONE, TURNING_LEFT, TURNING_RIGHT };
TurnState sharpTurnState = TURN_NONE;
unsigned long turnStartTime = 0;
const unsigned long SHARP_TURN_DURATION = 500; // increased — give it more time to complete

// ================= EXIT / JUNCTION DETECTION =================
bool handleJunction() {

  if(fabs(filteredError) > 0.7) return false;
  int leftCount  = IR_inner[0] + IR_inner[1] + IR_inner[2];
  int rightCount = IR_inner[3] + IR_inner[4] + IR_inner[5];
  int totalInner = leftCount + rightCount;

  if(sharpTurnState != TURN_NONE) return false;

  bool exitLeft  = (IR_left  == 1) && (totalInner >= 4) && rightCount >= 1;
  bool exitRight = (IR_right == 1) && (totalInner >= 4) && leftCount  >= 1;

  if(!exitLeft && !exitRight) return false;

  integral = 0;
  filteredError = 0;
  lastError = 0;

  if(exitRight){
    drive(160,-160);
    //delay(80);
    return true;
  }

  if(exitLeft){
    drive(-160,160);
    //delay(80);
    return true;
  }

  return false;
}

bool handleSharpTurn() {

  int leftCount  = IR_inner[0] + IR_inner[1] + IR_inner[2];
  int rightCount = IR_inner[3] + IR_inner[4] + IR_inner[5];
  int totalCount = leftCount + rightCount;

  if (anyInnerOnLine() && IR_left == 0 && IR_right == 0)
  return false;

  // ---- Detect entry into sharp turn ----
  if (sharpTurnState == TURN_NONE) {

    if (leftCount >= 2 && rightCount <= 1 && IR_right == 0) {
      sharpTurnState = TURNING_LEFT;
      turnStartTime  = millis();
      integral = 0; filteredError = 0; lastError = 0;
      return true;
    }

    if (rightCount >= 2 && leftCount <= 1 && IR_left == 0) {
      sharpTurnState = TURNING_RIGHT;
      turnStartTime  = millis();
      integral = 0; filteredError = 0; lastError = 0;
      return true;
    }

    return false;
  }

  // ---- Execute the maneuver ----
  unsigned long elapsed = millis() - turnStartTime;

  bool recentered = elapsed > 100 &&           // minimum turn time — can't exit before this
                    (IR_inner[2] || IR_inner[3]) &&
                    leftCount > 0 && rightCount > 0;

  if (recentered || elapsed > SHARP_TURN_DURATION) {
    sharpTurnState = TURN_NONE;
    integral = 0; filteredError = 0; lastError = 0;
    return false;
  }

  if (sharpTurnState == TURNING_LEFT) {
    drive(-160, 160);
  } else {
    drive(160, -160);
  }

  return true;
}

void pidControl(){

  readSensors();

  unsigned long now = micros();
  dt = (now - lastLoopTime) / 1000000.0f;
  dt = constrain(dt, 0.001f, 0.05f);
  lastLoopTime = now;

  float error = computeError();

  if(fabsf(error) < DEADBAND) error = 0.0f;

  filteredError = 0.2f * filteredError + 0.8f * error;

  if(handleSharpTurn()) return;
  if(handleJunction())  return;
  if(handleRecovery()) return;

  
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

  int adaptiveSpeed = baseSpeed - (int)(fabsf(filteredError) * SPEED_KP);
  adaptiveSpeed = constrain(adaptiveSpeed, MIN_CURVE_SPEED, baseSpeed);
  int activeSpeed = min(currentSpeed, adaptiveSpeed);

  int L = activeSpeed + (int)correction;
  int R = activeSpeed - (int)correction;

  int maxVal = max(abs(L), abs(R));
  if(maxVal > maxSpeed){
    L = (L * maxSpeed) / maxVal;
    R = (R * maxSpeed) / maxVal;
  }

  drive(L, R);
}

void loop(){
  if(running){
    handleSpeedRamp();
    pidControl();
  } else {
    stopMotors();
  }
}