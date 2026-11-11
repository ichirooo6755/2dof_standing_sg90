// XIAO nRF54L15 / SG90 commissioning. Software PWM reuses the isolation
// sketch's verified D0/D1 output method. Every boot starts with PWM off.
// c = center both, 1/2 = test D0/D1 by +15 and -15 degrees, ! = off.
// + / - trims upper center by 6 us toward positive / negative angle.
#include <Arduino.h>

static const uint8_t servoPins[2] = {PIN_D0, PIN_D1};
// 2026-11-11 operator confirmed both links straight at these commands.
static int centerUs[2] = {1500, 1464};
static const int testAmplitudeUs = 167;
static bool holding = false;
static int testing = -1;
static int pulsesUs[2] = {1500, 1500};
static uint32_t pulseCount[2] = {0, 0};
static uint32_t measuredHighUs[2] = {0, 0};
static uint32_t testStartedMs = 0;
static uint32_t lastFrameUs = 0;
static uint32_t reportAtMs = 0;

static void stopOutputs() {
  holding = false;
  testing = -1;
  for (int i = 0; i < 2; ++i) digitalWrite(servoPins[i], LOW);
}

static void centerOutputs() {
  pulsesUs[0] = centerUs[0];
  pulsesUs[1] = centerUs[1];
  testing = -1;
  holding = true;
  lastFrameUs = micros();
}

static void report() {
  if (!Serial || Serial.availableForWrite() < 160) return;
  Serial.print("CENTER holding="); Serial.print(holding ? 1 : 0);
  Serial.print(" testing="); Serial.print(testing + 1);
  Serial.print(" pulse_us="); Serial.print(holding ? pulsesUs[0] : 0);
  Serial.print(','); Serial.print(holding ? pulsesUs[1] : 0);
  Serial.print(" measured_high_us="); Serial.print(measuredHighUs[0]);
  Serial.print(','); Serial.print(measuredHighUs[1]);
  Serial.print(" count="); Serial.print(pulseCount[0]);
  Serial.print(','); Serial.println(pulseCount[1]);
}

void setup() {
  for (int i = 0; i < 2; ++i) {
    digitalWrite(servoPins[i], LOW);
    pinMode(servoPins[i], OUTPUT);
  }
  Serial.begin(115200);
}

void loop() {
  while (Serial.available()) {
    char command = (char)Serial.read();
    if (command == '!' || command == '0') {
      stopOutputs();
      Serial.println("OFF center outputs disabled");
    } else if (command == 'c') {
      centerOutputs();
      Serial.print("CENTER D0="); Serial.print(centerUs[0]);
      Serial.print(" D1="); Serial.print(centerUs[1]);
      Serial.println(" continuous_hold");
    } else if (command == '+' || command == '-') {
      int candidate = centerUs[1] + (command == '+' ? -6 : 6);
      centerUs[1] = constrain(candidate, 1444, 1556);
      centerOutputs();
      Serial.print("TRIM upper_center_us="); Serial.println(centerUs[1]);
    } else if (command == '1' || command == '2') {
      centerOutputs();
      testing = command - '1';
      testStartedMs = millis();
      Serial.print("TEST D"); Serial.print(testing);
      Serial.println(" +15,center,-15,center x3 then continuous center");
    } else if (command == 's') {
      report();
    }
  }

  uint32_t nowUs = micros();
  if (holding && uint32_t(nowUs - lastFrameUs) >= 20000) {
    if (uint32_t(nowUs - lastFrameUs) > 100000) {
      stopOutputs();
      Serial.println("FAULT frame overrun outputs disabled");
      return;
    }
    lastFrameUs = nowUs;
    pulsesUs[0] = centerUs[0];
    pulsesUs[1] = centerUs[1];
    if (testing >= 0) {
      uint32_t ageMs = millis() - testStartedMs;
      if (ageMs >= 19000) {
        testing = -1;
        Serial.println("DONE test continuous center hold");
      } else if (ageMs >= 1000) {
        unsigned phase = ((ageMs - 1000) / 1500) % 4;
        if (phase == 0) pulsesUs[testing] -= testAmplitudeUs;
        if (phase == 2) pulsesUs[testing] += testAmplitudeUs;
      }
    }
    // Both pulses are low before printing or receiving the next command.
    for (int i = 0; i < 2; ++i) {
      uint32_t riseUs = micros();
      digitalWrite(servoPins[i], HIGH);
      delayMicroseconds(pulsesUs[i]);
      digitalWrite(servoPins[i], LOW);
      measuredHighUs[i] = micros() - riseUs;
      ++pulseCount[i];
    }
  }
  if (uint32_t(millis() - reportAtMs) >= 500) {
    reportAtMs = millis();
    report();
  }
}
