// ================= DIGITAL IR =================
#define IR1 23
#define IR8 22

// ================= ANALOG IR =================
int IR_analog[6] = {36, 39, 34, 35, 32, 33};

// Adjust after observing raw values
int threshold = 85;

int mapped[8];

void setup() {

  Serial.begin(115200);

  pinMode(IR1, INPUT);
  pinMode(IR8, INPUT);

  Serial.println("IR Array Diagnostic Started...");
  Serial.println("--------------------------------");
}

void loop() {

  Serial.println("RAW VALUES:");

  // ===== DIGITAL 1 =====
  int d1 = digitalRead(IR1);
  Serial.print("IR1(D): ");
  Serial.print(d1);
  Serial.print("   ");

  // ===== ANALOG =====
  int analog_vals[6];

  for(int i=0;i<6;i++){
    analog_vals[i] = analogRead(IR_analog[i]);

    Serial.print("IR");
    Serial.print(i+2);
    Serial.print("(A): ");
    Serial.print(analog_vals[i]);
    Serial.print("   ");
  }

  // ===== DIGITAL 8 =====
  int d8 = digitalRead(IR8);
  Serial.print("IR8(D): ");
  Serial.println(d8);


  // ================= MAPPING =================

  // ⚠️ Digital sensors are often ACTIVE LOW
  // If reversed, just remove the !
  mapped[0] = d1;  

  for(int i=0;i<6;i++){
    mapped[i+1] = (analog_vals[i] > threshold) ? 1 : 0;
  }

  mapped[7] = d8;


  // ===== PRINT MAPPED =====
  Serial.print("MAPPED (1=BLACK,0=WHITE): ");

  for(int i=0;i<8;i++){
    Serial.print(mapped[i]);
    Serial.print(" ");
  }

  Serial.println();
  Serial.println("--------------------------------");

  delay(500); // slow for readability
}
