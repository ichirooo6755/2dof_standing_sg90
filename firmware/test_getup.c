#include <stdio.h>

#include "xiao_getup/getup.h"

static int failures = 0;

static void expect(int condition, const char *message) {
    if (!condition) {
        fprintf(stderr, "FAIL %s\n", message);
        failures++;
    }
}

static void test_attitude(void) {
    const float roll = 0.4f;
    const float pitch = -0.3f;
    const float ax = -cosf(roll) * sinf(pitch);
    const float ay = sinf(roll);
    const float az = cosf(roll) * cosf(pitch);
    float got_roll = 0.0f;
    float got_pitch = 0.0f;
    getup_attitude_from_accel(ax, ay, az, &got_roll, &got_pitch);
    expect(fabsf(got_roll - roll) < 1e-4f, "roll");
    expect(fabsf(got_pitch - pitch) < 1e-4f, "pitch");
}

static void test_fall_and_recover(void) {
    GetupState state;
    getup_init(&state);

    expect(!getup_poll(&state, 0.1f, 0.1f, 1000), "flat is not a fall");
    expect(state.mode == GETUP_MANUAL, "stays manual");

    expect(!getup_poll(&state, 1.2f, 0.0f, 2000), "fall hold has not elapsed");
    expect(!getup_poll(&state, 1.2f, 0.0f, 2299), "still inside 300 ms");
    expect(getup_poll(&state, 1.2f, 0.0f, 2301), "fall starts get-up");
    expect(state.mode == GETUP_RUNNING, "running");

    expect(getup_poll(&state, 0.1f, 0.1f, 2400), "upright hold not finished");
    expect(!getup_poll(&state, 0.1f, 0.1f, 2400 + 701), "upright ends get-up");
    expect(state.mode == GETUP_MANUAL, "manual after standing");

    expect(!getup_poll(&state, 1.2f, 0.0f, state.ended + 500), "cooldown blocks restart");
    uint32_t again = state.ended + GETUP_COOLDOWN_MS + 1;
    expect(!getup_poll(&state, 1.2f, 0.0f, again), "fall timer starts after cooldown");
    expect(getup_poll(&state, 1.2f, 0.0f, again + GETUP_FALL_HOLD_MS + 1),
           "falls again after cooldown");
}

static void test_button_stop_disables_auto(void) {
    GetupState state;
    getup_init(&state);
    getup_start(&state, 100);
    getup_stop(&state, 200, true);
    expect(!getup_poll(&state, 1.2f, 0.0f, 5000), "auto stays off");
    expect(state.mode == GETUP_MANUAL, "stopped");
}

static void test_servo_center(void) {
    expect(getup_servo_pulse_us(0.0f, 0) == 1500, "center pulse");
    expect(getup_servo_pulse_us(1.5707963f, 0) < 1500, "positive target moves one way");
    expect(getup_servo_pulse_us(-1.5707963f, 0) > 1500, "negative target moves the other way");
}

int main(void) {
    test_attitude();
    test_fall_and_recover();
    test_button_stop_disables_auto();
    test_servo_center();
    if (failures) {
        fprintf(stderr, "%d failed\n", failures);
        return 1;
    }
    puts("ok");
    return 0;
}
