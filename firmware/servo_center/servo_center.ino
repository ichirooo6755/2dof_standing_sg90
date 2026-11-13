// XIAO nRF54L15 or nRF54LM20A / SG90 commissioning. Every boot starts with PWM off.
// c = center both, 1/2 = test lower/upper by +15 and -15 degrees, ! = off.
// + / - trims upper center by 6 us toward positive / negative angle.
#include <Arduino.h>

#if defined(ARDUINO_NRF54LM20A)
static const char* boardName = "XIAO_nRF54LM20A";
static const char* upperPinName = "D2";
static const uint8_t servoPins[2] = {PIN_D0, PIN_D2};
#elif defined(ARDUINO_XIAO_NRF54L15_CLEAN)
static const char* boardName = "XIAO_nRF54L15";
static const char* upperPinName = "D1";
static const uint8_t servoPins[2] = {PIN_D0, PIN_D1};
#else
#error "Select the XIAO nRF54L15 or XIAO nRF54LM20A board in nrf54l15clean."
#endif

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

static void reportVersion() {
  Serial.print("SERVO_CENTER_2 board="); Serial.print(boardName);
  Serial.print(" lower=D0 upper="); Serial.print(upperPinName);
  Serial.println(" center_us=1500,1464 boot=OFF");
}

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
  reportVersion();
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
      Serial.print(' '); Serial.print(upperPinName); Serial.print('='); Serial.print(centerUs[1]);
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
      Serial.print("TEST "); Serial.print(testing == 0 ? "D0" : upperPinName);
      Serial.println(" +15,center,-15,center x3 then continuous center");
    } else if (command == 'v') {
      reportVersion();
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
