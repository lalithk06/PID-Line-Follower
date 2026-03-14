// ========================================
// LINE FOLLOWER v5.2
// ESP32 + TB6612FNG + 8 IR Sensors
// ========================================

// ================= MOTOR PINS =================
#define AIN1 17
#define AIN2 16
#define BIN1 27
#define BIN2 13
#define PWMA 25
#define PWMB 26
#define STBY 21

// ================= SENSOR PINS =================
#define IR1 23
#define IR8 22
int IR_analog[6] = {36, 39, 34, 35, 32, 33};

// ================= THRESHOLD =================
const int THRESHOLD = 600;

// ================= PID =================
float Kp = 2.0f;
float Ki = 0.0f;
float Kd = 2.5f;

float integral      = 0;
float lastError     = 0;
float filteredError = 0;

// ================= SENSOR POSITIONS =================
const float sensorPos[6] = {-3.75f, -2.25f, -0.75f, 0.75f, 2.25f, 3.75f};

// ================= SPEED =================
int baseSpeed = 200;
int maxSpeed  = 255;
int turnSpeed = 230;

int currentSpeed = 0;
const int rampStep  = 5;
const int rampDelay = 15;
unsigned long lastRampTime = 0;

// ✅ CHANGE 2: Scale reduced 20→12 for safer starting point
const float CORRECTION_SCALE = 12.0f;

// ================= SHARP TURN =================
bool inSharpTurn = false;
int  turnDir     = 0;
unsigned long turnStartTime = 0;
const unsigned long TURN_TIMEOUT = 2000;

// ================= DASHED LINE =================
unsigned long lastLineSeenTime         = 0;
const unsigned long DASHED_GAP_TIMEOUT = 300;
bool inDashedGap = false;

// ================= END ZONE =================
unsigned long allBlackStartTime = 0;
bool endZoneDetected = false;

// ================= RUNNING =================
bool running = false;

// ================= SENSOR DATA =================
int rawADC[6];
int IR_inner[6];
int IR_left  = 0;
int IR_right = 0;

// ================= dt TRACKING =================
// dt kept for I term only — D term no longer uses it
unsigned long lastLoopTime = 0;
float dt = 0.005f;

// ============================================================
// SETUP
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

  Serial.println("========================================");
  Serial.println("LINE FOLLOWER v5.2");
  Serial.println("6 analog sensors PID | 2 digital turn/edge");
  Serial.println("========================================");
  Serial.printf("Kp=%.2f Ki=%.3f Kd=%.2f BaseSpd=%d Scale=%.1f\n",
                Kp, Ki, Kd, baseSpeed, CORRECTION_SCALE);

  running           = true;
  currentSpeed      = 0;
  lastRampTime      = millis();
  lastLineSeenTime  = millis();
  lastLoopTime      = micros();
  integral          = 0;
  lastError         = 0;
  filteredError     = 0;
  endZoneDetected   = false;
  allBlackStartTime = 0;

  Serial.println("LINE FOLLOW STARTED");
}

// ============================================================
// MOTOR CONTROL
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
// READ SENSORS
// ============================================================
void readSensors(){

  IR_left  = (digitalRead(IR1) == HIGH) ? 1 : 0;
  IR_right = (digitalRead(IR8) == HIGH) ? 1 : 0;

  for(int i = 0; i < 6; i++){
    rawADC[i]   = analogRead(IR_analog[i]);
    IR_inner[i] = (rawADC[i] > THRESHOLD) ? 1 : 0;
  }

  for(int i = 0; i < 6; i++){
    if(IR_inner[i]){ lastLineSeenTime = millis(); break; }
  }
}

// ============================================================
// ANALOG WEIGHTED CENTROID — inner 6 sensors only
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

  return weightedSum / totalWeight;
}

// ============================================================
// END ZONE
// ============================================================
/*bool handleEndZone(){

  int blackCount = IR_left + IR_right;
  for(int i = 0; i < 6; i++) blackCount += IR_inner[i];

  if(blackCount == 8){
    if(allBlackStartTime == 0) allBlackStartTime = millis();
    if(millis() - allBlackStartTime >= 150){
      if(!endZoneDetected){
        endZoneDetected = true;
        Serial.println("END ZONE — STOPPING");
        stopMotors();
        running = false;
        return true;
      }
    }
    drive(baseSpeed / 3, baseSpeed / 3);
    return true;
  }

  allBlackStartTime = 0;
  return false;
}*/

// ============================================================
// DASHED LINE
// ============================================================
bool handleDashedLine(){

  bool innerSeen = false;
  for(int i = 0; i < 6; i++) if(IR_inner[i]){ innerSeen = true; break; }

  if(innerSeen){
    inDashedGap = false;
    return false;
  }

  unsigned long gap = millis() - lastLineSeenTime;

  if(gap < DASHED_GAP_TIMEOUT){
    if(!inDashedGap){
      inDashedGap = true;
      Serial.println("DASHED GAP — straight");
    }
    float correction = lastError * CORRECTION_SCALE;
    int L = constrain(currentSpeed + (int)correction, 0, maxSpeed);
    int R = constrain(currentSpeed - (int)correction, 0, maxSpeed);
    drive(L, R);
    return true;
  }

  return false;
}

// ============================================================
// SHARP TURN
// ============================================================
bool handleSharpTurn(){

  int leftSide  = IR_left  + IR_inner[0];
  int rightSide = IR_right + IR_inner[5];
  int center    = IR_inner[2] + IR_inner[3];

  if(!inSharpTurn){

    if(leftSide >= 2 && rightSide == 0){
      inSharpTurn   = true;
      turnDir       = -1;
      turnStartTime = millis();
      drive(0, 0);
      delay(40);
      integral      = 0;
      filteredError = 0;
      Serial.println("Sharp LEFT");
      return true;
    }

    if(rightSide >= 2 && leftSide == 0){
      inSharpTurn   = true;
      turnDir       = 1;
      turnStartTime = millis();
      drive(0, 0);
      delay(40);
      integral      = 0;
      filteredError = 0;
      Serial.println("Sharp RIGHT");
      return true;
    }

    return false;
  }

  bool timedOut = (millis() - turnStartTime > TURN_TIMEOUT);

  if((center >= 1 && leftSide <= 1 && rightSide <= 1) || timedOut){

    inSharpTurn = false;
    if(timedOut) Serial.println("Turn TIMEOUT");
    else         Serial.println("Turn complete");

    integral      = 0;
    filteredError = 0;
    lastError     = (turnDir == -1) ? -0.3f : 0.3f;
    turnDir       = 0;

    drive(currentSpeed, currentSpeed);
    return false;
  }

  if(turnDir == -1)
    drive(-(int)(turnSpeed * 0.7f), turnSpeed);
  else
    drive(turnSpeed, -(int)(turnSpeed * 0.7f));

  return true;
}

// ============================================================
// LINE LOST RECOVERY
// ============================================================
bool handleLineLost(){

  bool anyInner = false;
  for(int i = 0; i < 6; i++) if(IR_inner[i]){ anyInner = true; break; }
  if(anyInner) return false;

  if(lastError > 0)
    drive(turnSpeed, -turnSpeed);
  else
    drive(-turnSpeed, turnSpeed);

  return true;
}

// ============================================================
// SPEED RAMP
// ============================================================
void handleSpeedRamp(){
  if(millis() - lastRampTime >= rampDelay){
    lastRampTime = millis();
    if(currentSpeed < baseSpeed) currentSpeed += rampStep;
  }
}

// ============================================================
// HELPER
// ============================================================
bool onInnerLine(){
  for(int i = 0; i < 6; i++) if(IR_inner[i]) return true;
  return false;
}

// ============================================================
// PID CONTROL
// ============================================================
void pidControl(){

  readSensors();

  // dt for I term only
  unsigned long now = micros();
  dt = (now - lastLoopTime) / 1000000.0f;
  dt = constrain(dt, 0.001f, 0.05f);
  lastLoopTime = now;

 // if(handleEndZone())   return;
  if(handleDashedLine()) return;
  if(handleSharpTurn()) return;
  if(handleLineLost())  return;

  // ---- Normal PID ----

  float error = computeError();

  // ✅ CHANGE 4: Deadband — suppress micro-corrections near center
  // Prevents jitter from tiny sensor fluctuations when nearly straight
  // 0.08 cm is practical minimum — below this is noise, not real deviation
  if(fabsf(error) < 0.08f) error = 0.0f;

  // ✅ CHANGE 3: EMA alpha 0.7/0.3 → 0.5/0.5
  // Less history weight = faster response to real line position changes
  // Still smooths noise but without the lag that caused late overcorrection
  filteredError = 0.5f * filteredError + 0.5f * error;

  float P_term = Kp * filteredError;

  // ✅ CHANGE 1: D term — dt REMOVED
  // With dt=2ms, even 0.05 error change → D spike of 62.5 → motor jerk
  // Per-loop difference is stable and sufficient for a line follower
  // D term now means "change per loop" not "change per second"
  float D_term = Kd * (filteredError - lastError);

  // I term keeps dt — accumulates area under error curve correctly
  if(onInnerLine())
    integral += filteredError * dt;
  else
    integral = 0;
  integral = constrain(integral, -50.0f, 50.0f);
  float I_term = Ki * integral;

  float correction = (P_term + I_term + D_term) * CORRECTION_SCALE;

  lastError = filteredError;

  int L = currentSpeed + (int)correction;
  int R = currentSpeed - (int)correction;

  // Proportional saturation — preserve turn ratio
  int maxVal = max(abs(L), abs(R));
  if(maxVal > maxSpeed){
    L = (L * maxSpeed) / maxVal;
    R = (R * maxSpeed) / maxVal;
  }

  drive(L, R);

  static int dbg = 0;
  if(dbg++ % 8 == 0){
    Serial.printf("IR:%d%d|%d%d%d%d%d%d|%d%d rawE:%+.3f fE:%+.3f corr:%+.1f L:%3d R:%3d spd:%3d\n",
                  IR_left,  IR_inner[0],
                  IR_inner[1], IR_inner[2], IR_inner[3],
                  IR_inner[4], IR_inner[5], IR_right,
                  IR_left, IR_right,
                  error, filteredError, correction, L, R, currentSpeed);
  }
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop(){
  if(running){
    handleSpeedRamp();
    pidControl();
  } else {
    stopMotors();
  }
}