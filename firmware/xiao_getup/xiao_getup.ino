// XIAO nRF54L15 Sense firmware for the two-SG90 get-up robot.
// Board: XIAO nRF54L15 / Sense, lolren nrf54-arduino-core.
//
// Servo 1 (lower, on the foot) signal -> D0
// Servo 2 (upper) signal              -> D1
// Both servo grounds -> GND
// Both servo power   -> 5V
// The 5V pin is about 4.7 V from USB and the cell voltage from battery.
// Do not power the servos from 3V3. Give them their own 5 V supply, with
// the grounds tied together, if the board resets when a servo starts.

#include <Wire.h>

#include "getup.h"
#include "policy_network.h"

// Chip frame to MuJoCo foot frame. The board lies flat with USB pointing to
// the foot's -Y edge. This assumes the LSM6DS3 +Z faces up and its +X faces USB.
// Flat on the table should print az near +1. If it does not, change these signs.
static const float kAxisSign[3] = {1.0f, -1.0f, 1.0f};

static const uint8_t kServoPin[2] = {PIN_D0, PIN_D1};
static const int kServoTrimUs[2] = {0, 0};
static const uint8_t kImuAddress = 0x6A;
static const uint32_t kControlPeriodMs = 20;

static GetupState g_state;
static float g_roll = 0.0f;
static float g_pitch = 0.0f;
static bool g_imu_ok = false;
static bool g_attitude_ready = false;
static uint32_t g_last_control = 0;
static uint32_t g_last_report = 0;

static bool imu_write(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(kImuAddress);
    Wire1.write(reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

static bool imu_read(uint8_t reg, uint8_t *data, size_t length) {
    Wire1.beginTransmission(kImuAddress);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom(kImuAddress, (uint8_t)length) != length) return false;
    for (size_t i = 0; i < length; i++) data[i] = Wire1.read();
    return true;
}

static bool imu_begin(void) {
    pinMode(PIN_IMU_MIC_PWR, OUTPUT);
    digitalWrite(PIN_IMU_MIC_PWR, HIGH);  // TPS22916 enable is active high
    delay(10);
    Wire1.begin();

    uint8_t who = 0;
    if (!imu_read(0x0F, &who, 1) || who != 0x6A) {
        Serial.print("imu whoami failed: 0x");
        Serial.println(who, HEX);
        return false;
    }
    // Block updates, auto-increment, 104 Hz, ±2 g, ±245 dps.
    return imu_write(0x12, 0x44) && imu_write(0x10, 0x40) && imu_write(0x11, 0x40);
}

static bool imu_sample(float foot_acc[3], float foot_gyro[3]) {
    uint8_t raw[12];
    if (!imu_read(0x22, raw, sizeof(raw))) return false;

    float chip[6];
    for (int i = 0; i < 6; i++) {
        int16_t sample = (int16_t)((uint16_t)raw[i * 2] | ((uint16_t)raw[i * 2 + 1] << 8));
        chip[i] = (float)sample;
    }
    // gyro LSB = 8.75 mdps, accel LSB = 0.061 mg, at the ranges configured above.
    float gyro[3] = {
        chip[0] * 0.00875f * (3.14159265f / 180.0f),
        chip[1] * 0.00875f * (3.14159265f / 180.0f),
        chip[2] * 0.00875f * (3.14159265f / 180.0f),
    };
    float acc[3] = {chip[3] * 0.000061f, chip[4] * 0.000061f, chip[5] * 0.000061f};

    // [foot_x, foot_y, foot_z] = [chip_y, -chip_x, chip_z]
    foot_acc[0] = kAxisSign[0] * acc[1];
    foot_acc[1] = kAxisSign[1] * acc[0];
    foot_acc[2] = kAxisSign[2] * acc[2];
    foot_gyro[0] = kAxisSign[0] * gyro[1];
    foot_gyro[1] = kAxisSign[1] * gyro[0];
    foot_gyro[2] = kAxisSign[2] * gyro[2];
    return true;
}

static void pulse_servos(void) {
    float target[2] = {g_state.servo1, g_state.servo2};
    for (int i = 0; i < 2; i++) {
        int pulse = getup_servo_pulse_us(target[i], kServoTrimUs[i]);
        digitalWrite(kServoPin[i], HIGH);
        delayMicroseconds(pulse);
        digitalWrite(kServoPin[i], LOW);
    }
}

static bool button_edge(void) {
    static bool down = false;
    static uint32_t last = 0;
    bool pressed = digitalRead(PIN_BUTTON) == LOW;
    uint32_t now = millis();
    if (pressed && !down && now - last > 30) {
        down = true;
        last = now;
        return true;
    }
    if (!pressed) down = false;
    return false;
}

void setup(void) {
    Serial.begin(115200);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    for (int i = 0; i < 2; i++) pinMode(kServoPin[i], OUTPUT);

    getup_init(&g_state);
    g_imu_ok = imu_begin();
    g_last_control = millis();
    Serial.println(g_imu_ok ? "imu ok" : "imu failed");
}

void loop(void) {
    uint32_t now = millis();
    if (now - g_last_control < kControlPeriodMs) return;
    float dt = (now - g_last_control) * 0.001f;
    g_last_control = now;

    if (button_edge()) {
        if (g_state.mode == GETUP_RUNNING) getup_stop(&g_state, now, true);
        else getup_start(&g_state, now);
    }

    float acc[3] = {0, 0, 1};
    float gyro[3] = {0, 0, 0};
    bool sample_ok = g_imu_ok && imu_sample(acc, gyro);
    if (sample_ok) {
        float roll_acc, pitch_acc;
        getup_attitude_from_accel(acc[0], acc[1], acc[2], &roll_acc, &pitch_acc);
        if (!g_attitude_ready) {
            g_roll = roll_acc;
            g_pitch = pitch_acc;
            g_attitude_ready = true;
        } else {
            const float alpha = 0.98f;
            g_roll = alpha * (g_roll + gyro[0] * dt) + (1.0f - alpha) * roll_acc;
            g_pitch = alpha * (g_pitch + gyro[1] * dt) + (1.0f - alpha) * pitch_acc;
        }
    }

    if (g_attitude_ready && getup_poll(&g_state, g_roll, g_pitch, now)) {
        float obs[OBS_DIM] = {g_roll, g_pitch, g_state.servo1, g_state.servo2};
        float action[ACTION_DIM];
        forward_policy(obs, action);
        getup_apply_action(&g_state, action[0], action[1]);
    }

    pulse_servos();
    digitalWrite(LED_BUILTIN, g_state.mode == GETUP_RUNNING ? LOW : HIGH);

    if (now - g_last_report > 200) {
        g_last_report = now;
        Serial.print(g_state.mode == GETUP_RUNNING ? "GETUP" : "MANUAL");
        Serial.print(g_state.auto_getup ? " auto" : " hold");
        Serial.print(getup_fallen(g_roll, g_pitch) ? " fallen" : " upright");
        Serial.print(" roll=");
        Serial.print(g_roll, 3);
        Serial.print(" pitch=");
        Serial.print(g_pitch, 3);
        Serial.print(" servo=");
        Serial.print(g_state.servo1, 3);
        Serial.print(',');
        Serial.println(g_state.servo2, 3);
    }
}
