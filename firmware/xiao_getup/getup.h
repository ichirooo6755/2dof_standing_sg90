#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Same thresholds as the original M5Atom get-up sketch. */
#define GETUP_FALL_RAD 0.75f
#define GETUP_DONE_RAD 0.25f
#define GETUP_FALL_HOLD_MS 300u
#define GETUP_DONE_HOLD_MS 700u
#define GETUP_TIMEOUT_MS 14000u
#define GETUP_COOLDOWN_MS 1500u
#define GETUP_TARGET_DELTA 0.08f
#define GETUP_TARGET_LIMIT 1.55f

typedef enum { GETUP_MANUAL = 0, GETUP_RUNNING = 1 } GetupMode;

typedef struct {
    GetupMode mode;
    bool auto_getup;
    float servo1;
    float servo2;
    uint32_t fall_since;
    uint32_t upright_since;
    uint32_t started;
    uint32_t ended;
} GetupState;

static inline void getup_init(GetupState *state) {
    state->mode = GETUP_MANUAL;
    state->auto_getup = true;
    state->servo1 = 0.0f;
    state->servo2 = 0.0f;
    state->fall_since = 0;
    state->upright_since = 0;
    state->started = 0;
    state->ended = 0;
}

static inline float getup_clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static inline bool getup_fallen(float roll, float pitch) {
    return fabsf(roll) > GETUP_FALL_RAD || fabsf(pitch) > GETUP_FALL_RAD;
}

static inline bool getup_upright(float roll, float pitch) {
    return fabsf(roll) < GETUP_DONE_RAD && fabsf(pitch) < GETUP_DONE_RAD;
}

/* Accelerometer in the foot frame, in g. Recover the MuJoCo roll/pitch. */
static inline void getup_attitude_from_accel(float ax, float ay, float az,
                                             float *roll, float *pitch) {
    *roll = atan2f(ay, sqrtf(ax * ax + az * az));
    *pitch = atan2f(-ax, az);
}

static inline void getup_apply_action(GetupState *state, float action1, float action2) {
    action1 = getup_clampf(action1, -1.0f, 1.0f);
    action2 = getup_clampf(action2, -1.0f, 1.0f);
    state->servo1 = getup_clampf(state->servo1 + action1 * GETUP_TARGET_DELTA,
                                 -GETUP_TARGET_LIMIT, GETUP_TARGET_LIMIT);
    state->servo2 = getup_clampf(state->servo2 + action2 * GETUP_TARGET_DELTA,
                                 -GETUP_TARGET_LIMIT, GETUP_TARGET_LIMIT);
}

/* SG90 pulse. target 0 rad is mechanical center, 1500 us. */
static inline int getup_servo_pulse_us(float target_rad, int trim_us) {
    float degrees = getup_clampf(target_rad * (180.0f / 3.14159265f), -90.0f, 90.0f);
    float command = 90.0f - degrees;
    int pulse = (int)lroundf(1500.0f + (command - 90.0f) * (1000.0f / 90.0f)) + trim_us;
    if (pulse < 500) return 500;
    if (pulse > 2500) return 2500;
    return pulse;
}

static inline void getup_start(GetupState *state, uint32_t now) {
    state->auto_getup = true;
    state->servo1 = 0.0f;
    state->servo2 = 0.0f;
    state->mode = GETUP_RUNNING;
    state->started = now;
    state->upright_since = 0;
    state->fall_since = 0;
}

static inline void getup_stop(GetupState *state, uint32_t now, bool disable_auto) {
    state->mode = GETUP_MANUAL;
    state->ended = now;
    state->fall_since = 0;
    state->upright_since = 0;
    if (disable_auto) state->auto_getup = false;
}

/* Returns true when the policy should advance one 20 ms step. */
static inline bool getup_poll(GetupState *state, float roll, float pitch, uint32_t now) {
    if (state->mode == GETUP_RUNNING) {
        if (getup_upright(roll, pitch)) {
            if (state->upright_since == 0) state->upright_since = now;
            if (now - state->upright_since > GETUP_DONE_HOLD_MS) {
                getup_stop(state, now, false);
                return false;
            }
        } else {
            state->upright_since = 0;
        }
        if (now - state->started > GETUP_TIMEOUT_MS) {
            getup_stop(state, now, false);
            return false;
        }
        return true;
    }

    if (!state->auto_getup || now - state->ended < GETUP_COOLDOWN_MS) {
        state->fall_since = 0;
        return false;
    }
    if (!getup_fallen(roll, pitch)) {
        state->fall_since = 0;
        return false;
    }
    if (state->fall_since == 0) state->fall_since = now;
    if (now - state->fall_since > GETUP_FALL_HOLD_MS) getup_start(state, now);
    return state->mode == GETUP_RUNNING;
}
