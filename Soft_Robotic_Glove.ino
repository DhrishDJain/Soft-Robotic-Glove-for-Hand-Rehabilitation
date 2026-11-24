#include <Arduino.h>

// ════════════════════════════════════════════════════════════════
//  SOFT ROBOTIC GLOVE — ON/OFF, Leak Compensation + Intent Deflate
//  FSR Pins : 25(Thumb) 33(Index/Driver) 32(Middle) 35(Ring) 34(Pinky)
//  Relays   : GPIO12 = INFLATE | GPIO13 = DEFLATE
//
//  BEHAVIOUR:
//  ┌─ ADC > 100 and ≤ 3500  ──→ INFLATE (fresh grip or leak top-up)
//  ├─ ADC > 3500            ──→ HOLD    (both OFF, fully inflated)
//  ├─ ADC drops < 3500 (>0) ──→ INFLATE top-up (leak compensation)
//  └─ ADC = 0 for ≥ 1s      ──→ DEFLATE non-stop (intentional release)
//
//  KEY DISTINCTION:
//  Leak  → ADC drops GRADUALLY (never hits 0) → top-up inflate
//  Intent → ADC hits 0 suddenly and holds 1s   → deflate
// ════════════════════════════════════════════════════════════════

#define RELAY_INFLATE 12
#define RELAY_DEFLATE 13

#define FSR_THUMB 34
#define FSR_INDEX  32  // Primary driver
#define FSR_MIDDLE 35
#define FSR_RING 33
#define FSR_PINKY 25

// ── Thresholds ───────────────────────────────────────────────────
#define THRESH_TOUCH 100  // ADC > 100  → finger on object
#define THRESH_FULL 4050  // ADC > 3500 → fully inflated
#define THRESH_ZERO 5     // ADC ≤ 5    → treat as zero (noise floor)

// ── Intentional deflate: ADC must stay at 0 for this long ────────
#define DEFLATE_CONFIRM_MS 1000  // 1 second of zero = intentional

// ── Oversampling ─────────────────────────────────────────────────
#define OVERSAMPLE 16

// ════════════════════════════════════════════════════════════════
int readFSR(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < OVERSAMPLE; i++) sum += analogRead(pin);
  return (int)(sum / OVERSAMPLE);
}

void setInflate() {
  digitalWrite(RELAY_DEFLATE, LOW);
  delayMicroseconds(200);
  digitalWrite(RELAY_INFLATE, HIGH);
  Serial.println(">> INFLATE ON  — GPIO12 HIGH | GPIO13 LOW");
}

void setDeflate() {
  digitalWrite(RELAY_INFLATE, LOW);
  delayMicroseconds(200);
  digitalWrite(RELAY_DEFLATE, HIGH);
  Serial.println(">> DEFLATE ON  — GPIO13 HIGH | GPIO12 LOW");
}

void setBothOff() {
  digitalWrite(RELAY_INFLATE, LOW);
  digitalWrite(RELAY_DEFLATE, LOW);
}

// ── States ───────────────────────────────────────────────────────
enum RelayState { IDLE,
                  INFLATING,
                  HOLDING,
                  DEFLATING };
RelayState relayState = IDLE;

// ── Zero intent timer ─────────────────────────────────────────────
bool zeroTimerActive = false;
uint32_t zeroStartTime = 0;

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_INFLATE, OUTPUT);
  pinMode(RELAY_DEFLATE, OUTPUT);
  setBothOff();

  analogReadResolution(12);
  analogSetPinAttenuation(FSR_THUMB, ADC_11db);
  analogSetPinAttenuation(FSR_INDEX, ADC_11db);
  analogSetPinAttenuation(FSR_MIDDLE, ADC_11db);
  analogSetPinAttenuation(FSR_RING, ADC_11db);
  analogSetPinAttenuation(FSR_PINKY, ADC_11db);

  Serial.println("════════════════════════════════════════════════════");
  Serial.println(" Soft Robotic Glove — Leak Compensation + Intent Deflate");
  Serial.println(" Index FSR = GPIO33 | Inflate=GPIO12 | Deflate=GPIO13");
  Serial.printf(" Touch start   > ADC %d\n", THRESH_TOUCH);
  Serial.printf(" Fully inflated> ADC %d → HOLD\n", THRESH_FULL);
  Serial.printf(" Leak top-up   < ADC %d (not 0)\n", THRESH_FULL);
  Serial.printf(" Intent deflate= ADC 0 for > %dms\n", DEFLATE_CONFIRM_MS);
  Serial.println("════════════════════════════════════════════════════\n");
}

// ════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  int indexADC = readFSR(FSR_INDEX);
  int t = readFSR(FSR_THUMB);
  int m = readFSR(FSR_MIDDLE);
  int r = readFSR(FSR_RING);
  int p = readFSR(FSR_PINKY);

  bool isZero = (indexADC <= THRESH_ZERO);
  bool isTouch = (indexADC > THRESH_TOUCH);
  bool isFull = (indexADC > THRESH_FULL);

  Serial.printf("T:%4d IDX:%4d M:%4d R:%4d P:%4d | %-9s | zero:%s\n",
                t, indexADC, m, r, p,
                relayState == INFLATING ? "INFLATING" : relayState == HOLDING   ? "HOLDING"
                                                      : relayState == DEFLATING ? "DEFLATING"
                                                                                : "IDLE",
                zeroTimerActive ? "counting" : "no");

  // ════════════════════════════════════════════════════════════
  //  STEP 1 — ZERO TIMER (intentional deflate detection)
  //  Runs independent of state.
  //  ADC hits 0 → start 1s timer.
  //  ADC rises again → cancel timer immediately.
  // ════════════════════════════════════════════════════════════
  if (isZero) {
    if (!zeroTimerActive) {
      zeroTimerActive = true;
      zeroStartTime = now;
      Serial.printf("   [ZERO]  ADC=0. Deflating in %dms if held...\n",
                    DEFLATE_CONFIRM_MS);
    }
  } else {
    if (zeroTimerActive) {
      zeroTimerActive = false;
      Serial.println("   [ZERO]  Cancelled — ADC recovered (noise or brief lift).");
    }
  }

  // ════════════════════════════════════════════════════════════
  //  STEP 2 — TRIGGER INTENTIONAL DEFLATE
  //  1s of sustained zero → deflate non-stop.
  //  Return immediately; skip all inflate logic below.
  // ════════════════════════════════════════════════════════════
  if (zeroTimerActive && (now - zeroStartTime >= DEFLATE_CONFIRM_MS)) {
    if (relayState != DEFLATING) {
      setDeflate();
      relayState = DEFLATING;
      Serial.println(">> DEFLATING — Intentional release confirmed (1s zero).");
    }
    delay(20);
    return;  // Deflate runs non-stop until ADC recovers
  }

  // ════════════════════════════════════════════════════════════
  //  STEP 3 — EXIT DEFLATE when user touches again
  //  ADC rises above noise floor → stop deflating, go IDLE.
  // ════════════════════════════════════════════════════════════
  if (relayState == DEFLATING && !isZero) {
    setBothOff();
    relayState = IDLE;
    Serial.println(">> IDLE — Touch detected after deflate. Ready.");
  }

  // ════════════════════════════════════════════════════════════
  //  STEP 4 — INFLATE / HOLD / LEAK TOP-UP
  //
  //  ADC > 3500          → HOLD (pump off, fully inflated)
  //  ADC 101–3500        → INFLATE
  //    This single branch handles BOTH:
  //      (a) Fresh touch  → ADC rises from 0 through 100 to 3500
  //      (b) Air leak     → ADC slowly drifts down from 3500,
  //                         never hits 0, re-inflate kicks in
  //  ADC 6–100           → Hysteresis band, hold current relay state
  //  ADC ≤ 5 (zero)      → Handled above by zero timer
  // ════════════════════════════════════════════════════════════
  if (!isZero) {

    if (isFull) {
      // ── Fully inflated → stop pump ───────────────────────
      if (relayState == INFLATING) {
        setBothOff();
        relayState = HOLDING;
        Serial.println(">> HOLDING  — ADC > 3500. Fully inflated, pump OFF.");
      }

    } else if (isTouch) {
      // ── Touch zone (101–3500):
      //    Fresh grip coming up OR leak-caused drop → inflate/top-up
      if (relayState != INFLATING) {
        setInflate();
        relayState = INFLATING;

        // Distinguish fresh vs top-up for serial clarity
        if (relayState == HOLDING || relayState == IDLE) {
          Serial.println(">> INFLATING — Leak top-up: ADC dropped below 3500.");
        } else {
          Serial.println(">> INFLATING — Fresh touch detected.");
        }
      }
    }
    // ── Hysteresis band (6–100): do nothing, hold state ──
  }
}