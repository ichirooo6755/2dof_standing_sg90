#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Same thresholds as the original M5Atom get-up sketch. */
#define GETUP_FALL_RAD 0.75f
#define GETUP_DONE_RAD 0.25f
#define GETUP_DONE_SERVO_RAD 0.12f
#define GETUP_FALL_HOLD_MS 300u
#define GETUP_DONE_HOLD_MS 700u
#define GETUP_CENTER_HOLD_MS 500u
#define GETUP_CENTER_ABORT_RAD 0.35f
#define GETUP_CENTER_DELTA 0.01f
#define GETUP_TIMEOUT_MS 14000u
#define GETUP_CENTER_TIMEOUT_MS 6000u
#define GETUP_TOTAL_TIMEOUT_MS (GETUP_TIMEOUT_MS + GETUP_CENTER_TIMEOUT_MS)
#define GETUP_REFERENCE_WAYPOINT_MS 350u
#define GETUP_COOLDOWN_MS 1500u
#define GETUP_TARGET_DELTA 0.08f
#define GETUP_TARGET_LIMIT 1.55f

typedef enum { GETUP_MANUAL = 0, GETUP_RUNNING = 1 } GetupMode;

typedef struct {
    GetupMode mode;
    bool auto_getup;
    bool centering;
    bool recovery_selected;
    int8_t roll_reference;
    uint8_t reference_step;
    float servo1;
    float servo2;
    uint32_t fall_since;
    uint32_t upright_since;
    uint32_t level_since;
    uint32_t centering_started;
    uint32_t reference_since;
    float reference_from1;
    float reference_from2;
    uint32_t started;
    uint32_t ended;
} GetupState;

static inline void getup_init(GetupState *state) {
    state->mode = GETUP_MANUAL;
    /* Start unarmed; board startup arms after a healthy IMU sample. */
    state->auto_getup = false;
    state->centering = false;
    state->recovery_selected = false;
    state->roll_reference = 0;
    state->reference_step = 0;
    state->servo1 = 0.0f;
    state->servo2 = 0.0f;
    state->fall_since = 0;
    state->upright_since = 0;
    state->level_since = 0;
    state->centering_started = state->reference_since = 0;
    state->reference_from1 = state->reference_from2 = 0;
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
    /* Same Z-Y-X Euler convention as sim/robo1_env.py::quat_to_roll_pitch.
     * Gravity in the body frame is [-sin(p), cos(p)sin(r), cos(p)cos(r)]. */
    *roll = atan2f(ay, az);
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
}

static inline void getup_apply_action(GetupState *state, float action1, float action2) {
    action1 = getup_clampf(action1, -1.0f, 1.0f);
    action2 = getup_clampf(action2, -1.0f, 1.0f);
    state->servo1 = getup_clampf(state->servo1 + action1 * GETUP_TARGET_DELTA,
                                 -GETUP_TARGET_LIMIT, GETUP_TARGET_LIMIT);
    state->servo2 = getup_clampf(state->servo2 + action2 * GETUP_TARGET_DELTA,
                                 -GETUP_TARGET_LIMIT, GETUP_TARGET_LIMIT);
}

/* Final [0,0] waypoint from sim/getup_reference.py. The learned policy can
 * settle at a folded pose on hardware even with the foot flat. Return slowly
 * once the foot is stable; getup_poll hands back to the policy if it tips. */
static inline void getup_center_step(GetupState *state) {
    state->servo1 += getup_clampf(-state->servo1, -GETUP_CENTER_DELTA, GETUP_CENTER_DELTA);
    state->servo2 += getup_clampf(-state->servo2, -GETUP_CENTER_DELTA, GETUP_CENTER_DELTA);
}

/* Reuse sim/getup_reference.py's original GETUP_SEQUENCE and roll_neg path.
 * The MLP was trained on only one settled start for each roll direction;
 * shallow/combined real falls can produce zero or opposite first actions.
 * Latch the direction for one recovery instead of reclassifying every frame.
 * Left/right (pitch) falls keep using the existing learned controller. */
static inline void getup_select_recovery(GetupState *state, float roll, float pitch,
                                         uint32_t now) {
    if (state->recovery_selected) return;
    state->recovery_selected = true;
    /* Compare horizontal gravity, not Euler magnitudes: a side fall past
     * pitch=90 deg can have roll=180 deg without being a front/back fall. */
    float fore_aft = fabsf(cosf(pitch)*sinf(roll));
    float lateral = fabsf(sinf(pitch));
    state->roll_reference = getup_fallen(roll,pitch) && fore_aft>=lateral
                          ? (roll>=0 ? 1 : -1) : 0;
    state->reference_step = 0;
    state->reference_since = now;
    state->reference_from1 = state->servo1;
    state->reference_from2 = state->servo2;
}

static inline bool getup_reference_step(GetupState *state, uint32_t now) {
    static const float positive[][2] = {
        {1.532655f,-0.450321f}, {1.432846f,0.075935f},
        {-0.570885f,1.072732f}, {-0.713748f,0.631464f}, {0,0}
    };
    static const float negative[][2] = {{-1.55f,-1.55f}, {-1.55f,0}, {0,0}};
    if (!state->roll_reference) return false;
    const float (*points)[2] = state->roll_reference>0 ? positive : negative;
    unsigned count = state->roll_reference>0 ? 5u : 3u;
    if (state->reference_step>=count) return true;
    const float *goal = points[state->reference_step];
    float t = getup_clampf((float)(now-state->reference_since)/GETUP_REFERENCE_WAYPOINT_MS,0,1);
    float next1 = state->reference_from1 + t*(goal[0]-state->reference_from1);
    float next2 = state->reference_from2 + t*(goal[1]-state->reference_from2);
    state->servo1 = getup_clampf(state->servo1 + getup_clampf(next1-state->servo1,
                      -GETUP_TARGET_DELTA,GETUP_TARGET_DELTA),-GETUP_TARGET_LIMIT,GETUP_TARGET_LIMIT);
    state->servo2 = getup_clampf(state->servo2 + getup_clampf(next2-state->servo2,
                      -GETUP_TARGET_DELTA,GETUP_TARGET_DELTA),-GETUP_TARGET_LIMIT,GETUP_TARGET_LIMIT);
    if (t>=1 && fabsf(state->servo1-goal[0])<1e-5f && fabsf(state->servo2-goal[1])<1e-5f) {
        ++state->reference_step;
        state->reference_since = now;
        state->reference_from1 = state->servo1;
        state->reference_from2 = state->servo2;
    }
    return true;
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
    /* Preserve the last commanded pose; the policy observes these targets. */
    state->mode = GETUP_RUNNING;
    state->centering = false;
    state->recovery_selected = false;
    state->roll_reference = 0;
    state->started = now;
    state->upright_since = 0;
    state->level_since = 0;
    state->centering_started = 0;
    state->fall_since = 0;
}

static inline void getup_stop(GetupState *state, uint32_t now, bool disable_auto) {
    state->mode = GETUP_MANUAL;
    state->centering = false;
    state->ended = now;
    state->fall_since = 0;
    state->upright_since = 0;
    state->level_since = 0;
    if (disable_auto) state->auto_getup = false;
}

/* Returns true when recovery or final centering should advance a 20 ms step. */
static inline bool getup_poll(GetupState *state, float roll, float pitch, uint32_t now) {
    if (state->mode == GETUP_RUNNING) {
        if (state->centering && (fabsf(roll) > GETUP_CENTER_ABORT_RAD ||
                                 fabsf(pitch) > GETUP_CENTER_ABORT_RAD)) {
            state->centering = false;
            state->level_since = 0;
            state->recovery_selected = false;
        }
        getup_select_recovery(state,roll,pitch,now);
        if (getup_upright(roll, pitch)) {
            if (state->level_since == 0) state->level_since = now;
            if (!state->centering && now-state->level_since>=GETUP_CENTER_HOLD_MS) {
                state->centering = true;
                state->centering_started = now;
            }
        } else state->level_since = 0;
        /* A level foot is also possible with both arms still folded over.
         * PWM SG90s have no angle feedback: these are commanded angles only. */
        if (getup_upright(roll, pitch) &&
            fabsf(state->servo1) < GETUP_DONE_SERVO_RAD &&
            fabsf(state->servo2) < GETUP_DONE_SERVO_RAD) {
            if (state->upright_since == 0) state->upright_since = now;
            if (now - state->upright_since > GETUP_DONE_HOLD_MS) {
                getup_stop(state, now, false);
                return false;
            }
        } else {
            state->upright_since = 0;
        }
        if ((!state->centering && now-state->started>GETUP_TIMEOUT_MS) ||
            (state->centering && now-state->centering_started>GETUP_CENTER_TIMEOUT_MS) ||
            now-state->started>GETUP_TOTAL_TIMEOUT_MS) {
            /* Do not keep retrying a blocked or incorrectly assembled robot. */
            getup_stop(state, now, true);
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
    if (now - state->fall_since > GETUP_FALL_HOLD_MS) {
        getup_start(state, now);
        getup_select_recovery(state,roll,pitch,now);
    }
    return state->mode == GETUP_RUNNING;
}
