#include <stdio.h>

#include "xiao_getup/getup.h"
#include "xiao_getup/attitude.h"

static int failures = 0;

static void expect(int condition, const char *message) {
    if (!condition) {
        fprintf(stderr, "FAIL %s\n", message);
        failures++;
    }
}

static void test_attitude(void) {
    /* Independently rotate world +Z into the body using a Z-Y-X quaternion.
     * Include upside-down roll: the previous formula aliases this to upright. */
    const float rolls[] = {-2.8f, -1.6f, -0.4f, 0, 0.4f, 1.6f, 2.8f};
    const float pitches[] = {-1.4f, -0.3f, 0, 0.3f, 1.4f};
    for (unsigned r = 0; r < sizeof(rolls)/sizeof(*rolls); ++r) {
        for (unsigned p = 0; p < sizeof(pitches)/sizeof(*pitches); ++p) {
            float cr=cosf(rolls[r]/2), sr=sinf(rolls[r]/2);
            float cp=cosf(pitches[p]/2), sp=sinf(pitches[p]/2);
            float w=cr*cp, x=sr*cp, y=cr*sp, z=-sr*sp;
            float acc[3]={2*(x*z-w*y), 2*(w*x+y*z), 1-2*(x*x+y*y)};
            float roll, pitch;
            getup_attitude_from_accel(acc[0], acc[1], acc[2], &roll, &pitch);
            expect(fabsf(roll-rolls[r])<1e-4f, "quaternion roll convention");
            expect(fabsf(pitch-pitches[p])<1e-4f, "quaternion pitch convention");
            GetupAttitude q;
            attitude_reset(&q);
            const float gyro[3]={0,0,0};
            for (int i=0; i<100; ++i) expect(attitude_update(&q,acc,gyro,0.02f), "filter healthy");
            attitude_euler(&q,&roll,&pitch);
            expect(fabsf(roll-rolls[r])<1e-4f, "filter static roll");
            expect(fabsf(pitch-pitches[p])<1e-4f, "filter static pitch");
        }
    }
}

static void test_rotating_attitude(void) {
    GetupAttitude q;
    attitude_reset(&q);
    const float flat[3]={0,0,1}, zero[3]={0,0,0};
    expect(attitude_update(&q,flat,zero,0.02f), "filter initializes");
    for (int i=1; i<=150; ++i) {
        float angle=i*0.02f;
        float acc[3]={0,sinf(angle),cosf(angle)}, gyro[3]={1,0,0};
        expect(attitude_update(&q,acc,gyro,0.02f), "rotate past 90 degrees");
    }
    float r,p;
    attitude_euler(&q,&r,&p);
    expect(fabsf(r-3.0f)<0.025f && fabsf(p)<1e-4f, "full roll without false pitch");
    float bad[3]={NAN,0,1};
    expect(!attitude_update(&q,bad,zero,0.02f), "invalid sensor rejected");
    expect(!attitude_update(&q,flat,zero,0.2f), "stale integration rejected");
}

static void test_fall_and_recover(void) {
    GetupState state;
    getup_init(&state);

    expect(!state.auto_getup, "state initialization is unarmed until board arms it");
    expect(!getup_poll(&state, 1.2f, 0, 5000), "unarmed state cannot start motion");
    getup_start(&state, 10);
    getup_stop(&state, 20, false);

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

static void test_folded_arms_and_timeout(void) {
    GetupState state;
    getup_init(&state);
    state.servo1=-1.15f; state.servo2=-0.232f;
    getup_start(&state,100);
    expect(fabsf(state.servo1+1.15f)<1e-5f, "restart preserves target");
    expect(getup_poll(&state,0.148f,-0.099f,1000), "live folded pose still running");
    expect(getup_poll(&state,0.148f,-0.099f,2000), "level foot alone is not DONE");
    expect(state.centering, "stable foot selects final return to center");
    state.servo1=state.servo2=0;
    expect(getup_poll(&state,0,0,2100), "centered hold starts");
    expect(!getup_poll(&state,0,0,2801), "centered hold completes");
    getup_start(&state,3000);
    expect(!getup_poll(&state,1,0,3000+GETUP_TIMEOUT_MS+1), "timeout stops");
    expect(!state.auto_getup, "timeout disables retries");
    expect(!getup_poll(&state,1,0,50000), "timeout never restarts itself");
}

static void test_centering_recovery(void) {
    GetupState state;
    getup_init(&state);
    state.servo1=-1.15f; state.servo2=-0.232f;
    getup_start(&state,100);
    for (uint32_t now=120; now<6000 && state.mode==GETUP_RUNNING; now+=20) {
        if (getup_poll(&state,0.07f,0.01f,now) && state.centering) {
            float old=state.servo1;
            getup_center_step(&state);
            expect(fabsf(state.servo1-old)<=GETUP_CENTER_DELTA+1e-5f, "slow final return");
        }
    }
    expect(state.mode==GETUP_MANUAL, "folded live pose now converges to DONE");
    expect(fabsf(state.servo1)<1e-5f && fabsf(state.servo2)<1e-5f, "both targets finish at zero");
    expect(state.auto_getup, "successful return retains auto recovery");
    getup_start(&state,7000);
    state.servo1=-0.8f;
    getup_poll(&state,0,0,7020);
    getup_poll(&state,0,0,7520);
    expect(state.centering, "centering begins after stable hold");
    getup_poll(&state,0.5f,0,7540);
    expect(!state.centering && state.mode==GETUP_RUNNING, "tilt resumes policy");
    expect(fabsf(state.servo1+0.8f)<1e-5f, "handoff preserves current target");
}

static void test_button_stop_disables_auto(void) {
    GetupState state;
    getup_init(&state);
    getup_start(&state, 100);
    getup_stop(&state, 200, true);
    expect(!getup_poll(&state, 1.2f, 0.0f, 5000), "auto stays off");
    expect(state.mode == GETUP_MANUAL, "stopped");
}

static void test_roll_reference(void) {
    for (int sign=-1; sign<=1; sign+=2) {
        GetupState state;
        getup_init(&state);
        getup_start(&state,100);
        expect(getup_poll(&state,sign*1.2f,-0.2f,100), "roll recovery starts");
        expect(state.roll_reference==sign, "roll direction selects existing teacher path");
        getup_reference_step(&state,120);
        expect(sign*state.servo1>0, "first lower movement follows latched fall direction");
        getup_select_recovery(&state,-sign*1.2f,0.2f,140);
        expect(state.roll_reference==sign, "direction is not reclassified in mid trajectory");
        for (uint32_t now=140; now<=4000; now+=20) {
            float old1=state.servo1,old2=state.servo2;
            expect(getup_reference_step(&state,now), "roll reference handles step");
            expect(fabsf(state.servo1-old1)<=GETUP_TARGET_DELTA+1e-5f, "lower reference slew limit");
            expect(fabsf(state.servo2-old2)<=GETUP_TARGET_DELTA+1e-5f, "upper reference slew limit");
        }
        expect(fabsf(state.servo1)<1e-5f && fabsf(state.servo2)<1e-5f, "reference ends with both arms centered");
    }
    GetupState side;
    getup_init(&side); getup_start(&side,100);
    getup_poll(&side,0.1f,1.2f,100);
    expect(side.roll_reference==0 && !getup_reference_step(&side,120), "pitch falls retain learned policy");
    getup_start(&side,200);
    getup_poll(&side,3.1415f,1.48f,200);
    expect(side.roll_reference==0, "side fall past vertical is not mislabeled by Euler roll flip");
}

static void test_late_centering_timeout(void) {
    GetupState state;
    getup_init(&state); state.servo1=-1.55f;
    getup_start(&state,100);
    getup_poll(&state,1.2f,0,13000);
    getup_poll(&state,0.05f,0,13500);
    getup_poll(&state,0.05f,0,14000);
    expect(state.centering, "late return begins centering");
    for (uint32_t now=14020;now<19500 && state.mode==GETUP_RUNNING;now+=20) {
        if (getup_poll(&state,0.05f,0,now)) getup_center_step(&state);
    }
    expect(state.mode==GETUP_MANUAL && state.auto_getup, "late centering finishes past old 14 second deadline");
    expect(fabsf(state.servo1)<1e-5f, "late centering reaches zero");
    getup_start(&state,30000); state.servo1=-1.0f;
    getup_poll(&state,0,0,30100); getup_poll(&state,0,0,30600);
    expect(!getup_poll(&state,0,0,36601), "stuck centering has independent bounded timeout");
    expect(!state.auto_getup, "stuck centering still releases automatic recovery");
}

static void test_servo_center(void) {
    expect(getup_servo_pulse_us(0.0f, 0) == 1500, "center pulse");
    expect(getup_servo_pulse_us(1.5707963f, 0) < 1500, "positive target moves one way");
    expect(getup_servo_pulse_us(-1.5707963f, 0) > 1500, "negative target moves the other way");
}

int main(void) {
    test_attitude();
    test_rotating_attitude();
    test_fall_and_recover();
    test_button_stop_disables_auto();
    test_servo_center();
    test_folded_arms_and_timeout();
    test_centering_recovery();
    test_roll_reference();
    test_late_centering_timeout();
    if (failures) {
        fprintf(stderr, "%d failed\n", failures);
        return 1;
    }
    puts("ok");
    return 0;
}
