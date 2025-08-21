// Vehicle Startup + APPS Monitor
// Pins:
// Inputs:  A2 = BRK-IN, A0 = DASH, A4 = APPS_PRIMARY, A5 = APPS_SECONDARY
// Outputs: D2 = BUZZER (relay drive, HIGH = ON), D3 = RTD (relay drive), D4 = FAULT LED, D5 = MAIN RELAY (ON normally, off on APPS fault)

#define PIN_BRK_IN A2
#define PIN_DASH   A0
#define PIN_APPS1  A4
#define PIN_APPS2  A5

#define PIN_BUZZER 2    // D2
#define PIN_RTD    3    // D3
#define PIN_FAULT_LED 4 // D4
#define PIN_D5     5    // D5 (always on unless APPS fault)


// ---------- Configurable Parameters ----------
float thresh_BRK   = 2.0; // Volts threshold to consider BRK-IN "high"
float thresh_DASH  = 2.0; // Volts threshold to consider DASH "high"

float apps1_minV   = 0.5; // Primary APPS minimum voltage (mapped to 0%)
float apps1_maxV   = 4.0; // Primary APPS maximum voltage (mapped to 100%)
float apps2_minV   = 0.0; // Secondary APPS minimum voltage
float apps2_maxV   = 4.8; // Secondary APPS maximum voltage

float mismatch_limit = 100.0; // percent allowed difference between primary and secondary

// ---------- Error bitmask (bitwise) ----------
const uint8_t ERR_NONE      = 0x00;
const uint8_t ERR_MISMATCH  = 0x01; // bit0 - APPS percentage difference too large
const uint8_t ERR_LOW_VOLT  = 0x02; // bit1 - any APPS voltage below its min

// ---------- State Variables ----------
bool RTD = false;                          // latched RTD flag
bool buzzerTriggered = false;              // true once buzzer has been activated
unsigned long dashHighStart = 0;           // for 2s hold timing
uint8_t error_mask = ERR_NONE;             // store faults bitwise
bool apps_fault = false;

// ---------- Helpers ----------
float adcToVolts(int adcVal) {
  return (adcVal * 5.0) / 1023.0;
}

float appsToPercent(float voltage, float minV, float maxV) {
  if (voltage <= minV) return 0.0;
  if (voltage >= maxV) return 100.0;
  return (voltage - minV) * 100.0 / (maxV - minV);
}

void setOutputsForFault(bool faultPresent) {
  if (faultPresent) {
    digitalWrite(PIN_D5, LOW);
    digitalWrite(PIN_FAULT_LED, HIGH);
  } else {
    digitalWrite(PIN_D5, HIGH);
    digitalWrite(PIN_FAULT_LED, LOW);
  }
}

void printErrorMask(uint8_t mask) {
  Serial.print("ErrMask: 0x");
  if (mask < 16) Serial.print('0');
  Serial.print(mask, HEX);
  Serial.print(" [");
  if (mask == ERR_NONE) { Serial.print("NONE"); }
  else {
    bool first = true;
    if (mask & ERR_MISMATCH) { if (!first) Serial.print(","); Serial.print("MISMATCH"); first = false; }
    if (mask & ERR_LOW_VOLT) { if (!first) Serial.print(","); Serial.print("LOW_VOLT"); first = false; }
  }
  Serial.print("]");
}

// ---------- Setup ----------
void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RTD, OUTPUT);
  pinMode(PIN_FAULT_LED, OUTPUT);
  pinMode(PIN_D5, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RTD, LOW);
  digitalWrite(PIN_FAULT_LED, LOW);
  digitalWrite(PIN_D5, HIGH);

  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("=== Vehicle Startup & APPS Monitor ===");
}

// ---------- Main Loop ----------
void loop() {
  float v_brk  = adcToVolts(analogRead(PIN_BRK_IN));
  float v_dash = adcToVolts(analogRead(PIN_DASH));
  float v_apps1 = adcToVolts(analogRead(PIN_APPS1));
  float v_apps2 = adcToVolts(analogRead(PIN_APPS2));

  // -------- Vehicle Startup Sequence --------
  if (!RTD && !buzzerTriggered) {
    if (v_brk > thresh_BRK && v_dash > thresh_DASH) {
      if (dashHighStart == 0) dashHighStart = millis();
      if ((millis() - dashHighStart) >= 2000UL) {
        RTD = true;
        buzzerTriggered = true;             // buzzer will not trigger again
        digitalWrite(PIN_BUZZER, HIGH);     // turn on buzzer
        delay(500);                         // short buzzer duration
        digitalWrite(PIN_BUZZER, LOW);
        Serial.println(">> RTD ACTIVATED (One-time)");
      }
    } else {
      dashHighStart = 0;
    }
  }

  // -------- APPS Processing and Fault Checking --------
  float p_apps1 = appsToPercent(v_apps1, apps1_minV, apps1_maxV);
  float p_apps2 = appsToPercent(v_apps2, apps2_minV, apps2_maxV);
  float perc_diff = fabs(p_apps1 - p_apps2);

  error_mask = ERR_NONE;
  if (perc_diff > mismatch_limit) error_mask |= ERR_MISMATCH;
  if (v_apps1 < apps1_minV || v_apps2 < apps2_minV) error_mask |= ERR_LOW_VOLT;

  apps_fault = (error_mask != ERR_NONE);
  setOutputsForFault(apps_fault);

  // -------- Keep RTD on once triggered unless APPS fault --------
  if (apps_fault) {
    RTD = false;
  }

  // Output RTD state (D3 constant HIGH once latched, unless fault)
  digitalWrite(PIN_RTD, RTD ? HIGH : LOW);

  // -------- Serial Debug --------
  Serial.print("BRK: "); Serial.print(v_brk, 2); Serial.print("V  ");
  Serial.print("DASH: "); Serial.print(v_dash, 2); Serial.println("V");

  Serial.print("APPS1: "); Serial.print(v_apps1, 3); Serial.print("V (");
  Serial.print(p_apps1, 1); Serial.print("%)  ");
  Serial.print("APPS2: "); Serial.print(v_apps2, 3); Serial.print("V (");
  Serial.print(p_apps2, 1); Serial.print("%)  ");
  Serial.print("diff: "); Serial.print(perc_diff, 2); Serial.println("%");

  Serial.print("RTD: "); Serial.print(RTD ? "ON" : "OFF");
  Serial.print("  BUZZER_TRIGGERED: "); Serial.print(buzzerTriggered ? "YES" : "NO");
  Serial.print("  D5(MAIN): "); Serial.print(digitalRead(PIN_D5) ? "ON" : "OFF");
  Serial.print("  FAULT_LED: "); Serial.print(digitalRead(PIN_FAULT_LED) ? "ON" : "OFF");
  Serial.print("  ");
  printErrorMask(error_mask);
  Serial.println();
  Serial.println("--------------------------------------");

  delay(200);
}
