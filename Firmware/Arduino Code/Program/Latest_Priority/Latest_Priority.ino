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

const int THRESHOLD = 90;

float Kp = 39.5f;
float Ki = 0.0f;
float Kd = 8.8f;

float integral      = 0;
float lastError     = 0;
float filteredError = 0;

const float sensorPos[6] = {-3.5f, -2.0f, -0.5f, 0.5f, 2.0f, 3.5f};

int   baseSpeed       = 100;
int   maxSpeed        = 255;
int   MIN_CURVE_SPEED = 60;

const float SPEED_KP = 20.0f;

int currentSpeed = 0;
const int rampStep  = 10;
const int rampDelay = 10;
unsigned long lastRampTime = 0;

const float DEADBAND = 0.08f;

// ================= LOOP DETECTION =================
unsigned long curveStartTime   = 0;
int           curveDirection   = 0;
const unsigned long LOOP_TIMEOUT      = 800;  // reduced — edge count is primary detector
const int           LOOP_EDGE_THRESHOLD = 2;  // outer sensor must fire 2 times = loop
int  leftEdgeCount  = 0;
int  rightEdgeCount = 0;
int  prevIR_left    = 0;
int  prevIR_right   = 0;// reduced — junction logic catches most loops first

// ================= JUNCTION DEBOUNCE =================
int junctionDebounce = 0;
const int JUNCTION_DEBOUNCE_COUNT = 5;  // must see junction for 5 loops to confirm

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

  if(totalWeight < 10) return lastError;

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
      lastError     = 0;
      recoveryPhase = ON_LINE;
    }
    lastLineSeenTime = millis();
    return false;
  }

  // Line is lost — start recovery
  if(recoveryPhase == ON_LINE){
    spinDirection      = (lastError >= 0) ? 1 : -1;
    recoveryPhase      = SPINNING;
    recoveryPhaseStart = millis();
  }

  unsigned long phaseElapsed = millis() - recoveryPhaseStart;

  switch(recoveryPhase){

    case DASHED_GAP:
      {
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
        stopMotors();
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

// ============================================================
enum TurnState { TURN_NONE, TURNING_LEFT, TURNING_RIGHT };
TurnState sharpTurnState = TURN_NONE;
unsigned long turnStartTime = 0;
const unsigned long SHARP_TURN_DURATION = 500;

// ============================================================
// Junction with debounce + Left → Straight → Right priority
// ============================================================
bool handleJunction(int activeSpeed) {

  // Must be centered to consider junction
  if(fabs(filteredError) > 0.3f){
    junctionDebounce = 0;
    return false;
  }

  int leftCount  = IR_inner[0] + IR_inner[1] + IR_inner[2];
  int rightCount = IR_inner[3] + IR_inner[4] + IR_inner[5];
  int totalInner = leftCount + rightCount;

  if(sharpTurnState != TURN_NONE){
    junctionDebounce = 0;
    return false;
  }

  bool exitLeft   = (IR_left  == 1) && (totalInner >= 4) && rightCount >= 1;
  bool exitRight  = (IR_right == 1) && (totalInner >= 4) && leftCount  >= 1;
  bool straight   = (IR_inner[2] || IR_inner[3]);  // center sensors active

  if(!exitLeft && !exitRight){
    junctionDebounce = 0;  // reset if nothing seen
    return false;
  }

  // Debounce — must see junction for N consecutive loops
  junctionDebounce++;
  if(junctionDebounce < JUNCTION_DEBOUNCE_COUNT) return false;
  junctionDebounce = 0;  // reset after confirmed

  integral = 0; filteredError = 0; lastError = 0;

  int pivotSpeed = max(activeSpeed, 140);

  // ── LEFT → STRAIGHT → RIGHT priority ──
  if(exitLeft){
    drive(-pivotSpeed, pivotSpeed);
    return true;
  }
  if(straight){
    return false;  // let PID handle straight
  }
  if(exitRight){
    drive(pivotSpeed, -pivotSpeed);
    return true;
  }

  return false;
}

// ============================================================
bool handleSharpTurn() {

  int leftCount  = IR_inner[0] + IR_inner[1] + IR_inner[2];
  int rightCount = IR_inner[3] + IR_inner[4] + IR_inner[5];

  if(anyInnerOnLine() && IR_left == 0 && IR_right == 0)
    return false;

  if(sharpTurnState == TURN_NONE){

    if(leftCount >= 2 && rightCount <= 1 && IR_right == 0){
      sharpTurnState = TURNING_LEFT;
      turnStartTime  = millis();
      integral = 0; filteredError = 0; lastError = 0;
      return true;
    }

    if(rightCount >= 2 && leftCount <= 1 && IR_left == 0){
      sharpTurnState = TURNING_RIGHT;
      turnStartTime  = millis();
      integral = 0; filteredError = 0; lastError = 0;
      return true;
    }

    return false;
  }

  unsigned long elapsed = millis() - turnStartTime;

  bool recentered = elapsed > 100 &&
                    (IR_inner[2] || IR_inner[3]) &&
                    leftCount > 0 && rightCount > 0;

  if(recentered || elapsed > SHARP_TURN_DURATION){
    sharpTurnState = TURN_NONE;
    integral = 0; filteredError = 0; lastError = 0;
    return false;
  }

  if(sharpTurnState == TURNING_LEFT){
    drive(-160, 160);
  } else {
    drive(160, -160);
  }

  return true;
}

// ============================================================
void pidControl(){

  readSensors();

  // NEW — add these lines immediately after readSensors()
  if(IR_left  == 1 && prevIR_left  == 0) leftEdgeCount++;
  if(IR_right == 1 && prevIR_right == 0) rightEdgeCount++;
  prevIR_left  = IR_left;
  prevIR_right = IR_right;

  unsigned long now = micros();
  dt = (now - lastLoopTime) / 1000000.0f;
  dt = constrain(dt, 0.001f, 0.05f);
  lastLoopTime = now;

  // ── Compute error and activeSpeed FIRST ──
  float error = computeError();
  if(fabsf(error) < DEADBAND) error = 0.0f;
  filteredError = 0.2f * filteredError + 0.8f * error;

  int adaptiveSpeed = baseSpeed - (int)(fabsf(filteredError) * SPEED_KP);
  adaptiveSpeed = constrain(adaptiveSpeed, MIN_CURVE_SPEED, baseSpeed);
  int activeSpeed = min(currentSpeed, adaptiveSpeed);

  // ── Special handlers ──
  if(handleSharpTurn())           return;
  if(handleJunction(activeSpeed)) return;
  if(handleRecovery())            return;

  // ── Normal PID ──
  float P_term = Kp * filteredError;
  float D_term = Kd * (filteredError - lastError);

  if(anyInnerOnLine())
    integral += filteredError * dt;
  else
    integral = 0;
  integral = constrain(integral, -30.0f, 30.0f);
  float I_term = Ki * integral;

  float correction = P_term + I_term + D_term;

  // ── Update lastError BEFORE loop detection so D_term is correct next loop ──
  lastError = filteredError;

  // ── Loop detection — runs after PID terms calculated ──
  // ── Loop detection ──
  int turnDir = 0;
  if(filteredError >  1.0f) turnDir =  1;
  else if(filteredError < -1.0f) turnDir = -1;

  if(turnDir != 0){
    if(curveDirection == turnDir){
      if((millis() - curveStartTime > LOOP_TIMEOUT) &&
   ((turnDir ==  1 && rightEdgeCount >= LOOP_EDGE_THRESHOLD) ||
    (turnDir == -1 && leftEdgeCount  >= LOOP_EDGE_THRESHOLD))){
        int leftCount  = IR_inner[0] + IR_inner[1] + IR_inner[2];
        int rightCount = IR_inner[3] + IR_inner[4] + IR_inner[5];
        bool straight  = (IR_inner[2] || IR_inner[3]);

        if(IR_left == 1){
          drive(-160, 160);
        } else if(straight){
          // PID takes over
        } else if(IR_right == 1){
          drive(160, -160);
        } else {
          drive(160 * turnDir, -160 * turnDir);
        }

        curveDirection = 0;
        curveStartTime = millis();
        leftEdgeCount   = 0;   // ← add
        rightEdgeCount  = 0;  
        return;
      }
    } else {
      curveDirection = turnDir;   // ← was missing this line too!
      curveStartTime = millis();
      leftEdgeCount   = 0;   // ← add
      rightEdgeCount  = 0;  
    }
  } else {
    curveDirection = 0;
  }                               // ← closes if(turnDir != 0)

  // ── Motor output — always runs ──
  int L = activeSpeed + (int)correction;
  int R = activeSpeed - (int)correction;

  int maxVal = max(abs(L), abs(R));
  if(maxVal > maxSpeed){
    L = (L * maxSpeed) / maxVal;
    R = (R * maxSpeed) / maxVal;
  }

  drive(L, R);
}                                 // ← closes pidControl()

// ============================================================
void loop(){
  if(running){
    handleSpeedRamp();
    pidControl();
  } else {
    stopMotors();
  }
}
