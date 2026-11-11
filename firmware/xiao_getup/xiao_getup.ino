// XIAO nRF54L15 Sense, lolren nrf54-arduino-core 1.0.23.
// Lower/foot SG90 signal D0, upper SG90 D1; all grounds common.
// Servos need a suitable supply. The board 5V pin does not boost a Li-ion cell.
// Automatically arm fall recovery after the first healthy IMU sample.
// Button/off still stops outputs and disables automatic recovery until restarted.
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "getup.h"
#include "attitude.h"
#include "hardware_config.h"
#include "policy_network.h"

static const uint8_t kImuAddress = 0x6A;
static const uint32_t kControlPeriodMs = 20;
static GetupState g_state;
static GetupAttitude g_attitude;
static float g_roll = 0, g_pitch = 0;
static float g_acc[3] = {0}, g_gyro[3] = {0}, g_chip_acc[3] = {0};
static int g_axes[3], g_direction[2], g_trim[2];
static bool g_enabled[2] = {false, false};
static int g_pulse[2] = {1500, 1500};
static int g_probe_channel = -1;
static float g_probe_rad = 0;
static uint32_t g_probe_until = 0;
static bool g_imu_ok = false, g_sample_ok = false, g_report = true;
static bool g_boot_autostart_pending = true;
static const char *g_stop_reason = "BOOT_WAIT";
static uint32_t g_stop_ms = 0;
static uint8_t g_who = 0;
static uint32_t g_last_control = 0, g_last_sample = 0, g_last_report = 0;
static uint32_t g_last_retry = 0, g_imu_errors = 0, g_frame_ms = 0;
static char g_line[64];
static size_t g_line_len = 0;
static bool g_line_overflow = false;
static bool g_line_abort = false;

static bool imu_write(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(kImuAddress);
    Wire1.write(reg); Wire1.write(value);
    return Wire1.endTransmission() == 0;
}
static bool imu_read(uint8_t reg, uint8_t *data, size_t length) {
    Wire1.beginTransmission(kImuAddress); Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom(kImuAddress, (uint8_t)length) != length) return false;
    for (size_t i = 0; i < length; ++i) data[i] = Wire1.read();
    return true;
}
static bool imu_begin(void) {
    pinMode(PIN_IMU_MIC_PWR, OUTPUT);
    digitalWrite(PIN_IMU_MIC_PWR, HIGH);
    delay(40);
    Wire1.begin(); Wire1.setClock(400000);
    g_who = 0;
    if (!imu_read(0x0F, &g_who, 1) || g_who != 0x6A) return false;
    if (!imu_write(0x12, 0x01)) return false; // software reset
    uint8_t ctrl = 1;
    for (int i = 0; i < 20; ++i) {
        delay(2);
        if (!imu_read(0x12, &ctrl, 1)) return false;
        if (!(ctrl & 1)) break;
    }
    if (ctrl & 1) return false;
    // BDU + address increment; 104 Hz, +/-4 g and +/-2000 dps.
    if (!imu_write(0x12, 0x44) || !imu_write(0x10, 0x48) ||
        !imu_write(0x11, 0x4C)) return false;
    uint8_t verify[3];
    return imu_read(0x10, verify, 3) && verify[0] == 0x48 &&
           verify[1] == 0x4C && verify[2] == 0x44;
}
// 1=new sample, 0=not ready, -1=communication failure.
static int imu_sample(void) {
    uint8_t status, raw[12];
    if (!imu_read(0x1E, &status, 1)) return -1;
    if ((status & 3) != 3) return 0;
    if (!imu_read(0x22, raw, sizeof(raw))) return -1;
    float chip_gyro[3];
    for (int i = 0; i < 6; ++i) {
        int16_t v = (int16_t)((uint16_t)raw[2*i] | ((uint16_t)raw[2*i+1] << 8));
        if (i < 3) chip_gyro[i] = v * 0.070f * (3.14159265f/180.0f);
        else g_chip_acc[i-3] = v * 0.000122f;
    }
    for (int i = 0; i < 3; ++i) {
        int j = abs(g_axes[i])-1;
        float sign = g_axes[i] > 0 ? 1.0f : -1.0f;
        g_acc[i] = sign*g_chip_acc[j];
        g_gyro[i] = sign*chip_gyro[j];
    }
    return 1;
}
static void stop_outputs(uint32_t now, const char *reason) {
    g_boot_autostart_pending = false;
    g_stop_reason = reason;
    g_stop_ms = now;
    getup_stop(&g_state, now, true);
    g_probe_channel = -1;
    for (int i = 0; i < 2; ++i) {
        g_enabled[i] = false;
        digitalWrite(kServoPin[i], LOW);
    }
}
static void outputs_off(uint32_t now) { stop_outputs(now,"USER"); }
static int servo_pulse(int i, float target) {
    int p = (int)lroundf(1500.0f + g_trim[i] + g_direction[i] * target *
                       (180.0f/3.14159265f) * kServoUsPer90[i]/90.0f);
    return p < kServoMinUs[i] ? kServoMinUs[i] :
           p > kServoMaxUs[i] ? kServoMaxUs[i] : p;
}
static void pulse_servos(void) {
    for (int i = 0; i < 2; ++i) {
        if (!g_enabled[i]) continue;
        digitalWrite(kServoPin[i], HIGH);
        delayMicroseconds(g_pulse[i]);
        digitalWrite(kServoPin[i], LOW);
    }
}
static bool button_edge(uint32_t now) {
    static bool stable = false, candidate = false;
    static uint32_t changed = 0;
    bool pressed = digitalRead(PIN_BUTTON) == LOW;
    if (pressed != candidate) { candidate = pressed; changed = now; }
    if (candidate != stable && now-changed >= 40) {
        stable = candidate;
        return stable;
    }
    return false;
}
static bool imu_healthy(uint32_t now) {
    return g_imu_ok && g_sample_ok && g_attitude.ready && now-g_last_sample <= 60;
}
static void start_motion(uint32_t now) {
    if (!imu_healthy(now)) { Serial.println("ERR fresh IMU required"); return; }
    if (g_probe_channel >= 0) { Serial.println("ERR off before run"); return; }
    // Preserve the last commanded pose when resuming after off. On boot the
    // initialized targets are zero; SG90 actual position cannot be read.
    g_enabled[0] = g_enabled[1] = true;
    g_stop_reason = "NONE";
    getup_start(&g_state, now);
    // A supported, level foot needs only a slow return to center. Do not
    // feed a folded pose into the learned falling controller for 500 ms.
    if (getup_upright(g_roll,g_pitch)) {
        g_state.centering = true;
        g_state.centering_started = g_state.level_since = now;
    }
    Serial.println("OK run (button/off stops)");
}
static void report_status(uint32_t now) {
    Serial.print("STATUS ms="); Serial.print(now);
    Serial.print(" mode=");
    Serial.print(g_state.mode == GETUP_RUNNING ? "GETUP" :
                 g_probe_channel >= 0 ? "PROBE" :
                 g_enabled[0] || g_enabled[1] ? "HOLD" : "OFF");
    Serial.print(" phase=");
    Serial.print(g_state.mode!=GETUP_RUNNING ? "IDLE" : g_state.centering ? "CENTER" :
                 g_state.roll_reference>0 ? "ROLL_POS_REF" :
                 g_state.roll_reference<0 ? "ROLL_NEG_REF" : "POLICY");
    Serial.print(" ref_step="); Serial.print(g_state.reference_step);
    Serial.print(" auto="); Serial.print(g_state.auto_getup ? "ON" : "OFF");
    Serial.print(" boot_pending="); Serial.print(g_boot_autostart_pending ? 1 : 0);
    Serial.print(" imu="); Serial.print(imu_healthy(now) ? "OK" : "WAIT/FAULT");
    Serial.print(" who=0x"); Serial.print(g_who, HEX);
    Serial.print(" roll_deg="); Serial.print(g_roll*57.29578f, 2);
    Serial.print(" pitch_deg="); Serial.print(g_pitch*57.29578f, 2);
    Serial.print(" chip_g=");
    for (int i = 0; i < 3; ++i) { if (i) Serial.print(','); Serial.print(g_chip_acc[i], 3); }
    Serial.print(" foot_g=");
    for (int i = 0; i < 3; ++i) { if (i) Serial.print(','); Serial.print(g_acc[i], 3); }
    Serial.print(" target_deg="); Serial.print(g_state.servo1*57.29578f, 2);
    Serial.print(','); Serial.print(g_state.servo2*57.29578f, 2);
    Serial.print(" pulse_us="); Serial.print(g_enabled[0] ? g_pulse[0] : 0);
    Serial.print(','); Serial.print(g_enabled[1] ? g_pulse[1] : 0);
    Serial.print(" errors="); Serial.print(g_imu_errors);
    Serial.print(" stop="); Serial.print(g_stop_reason);
    Serial.print(" stop_ms="); Serial.print(g_stop_ms);
    Serial.print(" frame_ms="); Serial.println(g_frame_ms);
}
static bool valid_axes(int x, int y, int z) {
    int a[3] = {x,y,z};
    for (int i = 0; i < 3; ++i) if (abs(a[i]) < 1 || abs(a[i]) > 3) return false;
    if (abs(x)==abs(y) || abs(x)==abs(z) || abs(y)==abs(z)) return false;
    int inversions = (abs(x)>abs(y)) + (abs(x)>abs(z)) + (abs(y)>abs(z));
    int sign = (x<0 ? -1:1)*(y<0 ? -1:1)*(z<0 ? -1:1);
    return sign * (inversions%2 ? -1:1) == 1; // right-handed rotation only
}
static void command(const char *line, uint32_t now) {
    int channel, value, x, y, z;
    int degrees;
    char extra;
    bool stopped = !g_enabled[0] && !g_enabled[1];
    if (!strcmp(line, "off")) { outputs_off(now); Serial.println("OK off"); }
    else if (!strcmp(line, "status")) report_status(now);
    else if (!strcmp(line, "quiet")) { g_report = false; Serial.println("OK quiet"); }
    else if (!strcmp(line, "watch")) { g_report = true; Serial.println("OK watch"); }
    else if (!strcmp(line, "run")) start_motion(now);
    else if (sscanf(line, "restore_pose %d %d %c", &x, &y, &extra)==2 &&
             x>=-88 && x<=88 && y>=-88 && y<=88 && stopped &&
             g_state.mode!=GETUP_RUNNING && !g_state.auto_getup) {
        // Bench recovery after flashing: use previously logged COMMANDS.
        // This changes bookkeeping only; outputs remain OFF until run.
        g_state.servo1=x*(3.14159265f/180.0f);
        g_state.servo2=y*(3.14159265f/180.0f);
        Serial.print("OK restored command pose deg="); Serial.print(x);
        Serial.print(','); Serial.print(y); Serial.println(" outputs=OFF (not angle feedback)");
    }
    else if (!strcmp(line, "selfcheck")) {
        // Exercise the old parser on the actual embedded C library, without
        // driving a servo. newlib-nano can count a float conversion but leave
        // its destination unchanged when _scanf_float is not linked.
        int parsed_channel = 0;
        float old_degrees = 12345.0f;
        int n = sscanf("jog 1 10", "jog %d %f", &parsed_channel, &old_degrees);
        Serial.print("SELFCHECK old_float_count="); Serial.print(n);
        Serial.print(" old_float_value="); Serial.print(old_degrees, 3);
        Serial.print(" expected=10 old_parser=");
        Serial.println(n == 2 && old_degrees == 10.0f ? "OK" : "BROKEN");
        int new_degrees = 12345;
        n = sscanf("jog 1 10", "jog %d %d", &parsed_channel, &new_degrees);
        Serial.print("SELFCHECK integer_value="); Serial.print(new_degrees);
        Serial.print(" expected=10 new_parser=");
        Serial.println(n == 2 && new_degrees == 10 ? "OK" : "BROKEN");
    }
    else if (!strcmp(line, "config")) {
        Serial.print("CONFIG axes=");
        for (int i = 0; i < 3; ++i) { if (i) Serial.print(','); Serial.print(g_axes[i]); }
        Serial.print(" dir="); Serial.print(g_direction[0]); Serial.print(','); Serial.print(g_direction[1]);
        Serial.print(" trim_us="); Serial.print(g_trim[0]); Serial.print(','); Serial.println(g_trim[1]);
    }
    else if (sscanf(line, "center %d %c", &channel, &extra)==1 && channel>=1 && channel<=2) {
        if (g_state.mode==GETUP_RUNNING || g_state.auto_getup) { Serial.println("ERR off first"); return; }
        outputs_off(now);
        g_probe_channel = channel-1; g_probe_rad = 0;
        g_pulse[channel-1] = servo_pulse(channel-1, 0);
        g_enabled[channel-1] = true; g_probe_until = now+5000;
        Serial.println("OK center one channel; release after 5s");
    }
    else if (sscanf(line, "jog %d %d %c", &channel, &degrees, &extra)==2 &&
             channel>=1 && channel<=2 && degrees>=-10 && degrees<=10) {
        if (g_probe_channel!=channel-1 || !g_enabled[channel-1]) { Serial.println("ERR center this channel first"); return; }
        g_probe_rad = degrees*(3.14159265f/180.0f);
        g_probe_until = now+2000;
        Serial.print("OK absolute jog channel="); Serial.print(channel);
        Serial.print(" deg="); Serial.print(degrees);
        Serial.print(" target_us="); Serial.print(servo_pulse(channel-1,g_probe_rad));
        Serial.println("; release after 2s");
    }
    else if (sscanf(line, "dir %d %d %c", &channel, &value, &extra)==2 &&
             channel>=1 && channel<=2 && (value==1 || value==-1) && stopped) {
        g_direction[channel-1]=value; Serial.println("OK dir (RAM only)");
    }
    else if (sscanf(line, "trim %d %d %c", &channel, &value, &extra)==2 &&
             channel>=1 && channel<=2 && abs(value)<=200 && stopped) {
        g_trim[channel-1]=value; Serial.println("OK trim (RAM only)");
    }
    else if (sscanf(line, "axes %d %d %d %c", &x, &y, &z, &extra)==3 &&
             valid_axes(x,y,z) && stopped) {
        g_axes[0]=x; g_axes[1]=y; g_axes[2]=z;
        attitude_reset(&g_attitude); g_sample_ok=false;
        Serial.println("OK axes (RAM only)");
    }
    else {
        Serial.print("ERR received=["); Serial.print(line);
        Serial.println("] commands: status config selfcheck watch quiet off center N jog N INTEGER_DEG dir N SIGN trim N US axes X Y Z run");
    }
}
static void serial_commands(uint32_t now) {
    for (int n = 0; n < 64 && Serial.available(); ++n) {
        char c = (char)Serial.read();
        // Single-character emergency stop works even without a newline.
        if (c=='!') { outputs_off(now); g_line_len=0; g_line_abort=true; Serial.println("OK off"); continue; }
        if (g_line_abort) {
            if (c=='\r' || c=='\n') { g_line_abort=false; g_line_overflow=false; }
            continue;
        }
        if (c=='\r' || c=='\n') {
            if (g_line_overflow) Serial.println("ERR line too long");
            else if (g_line_len) { g_line[g_line_len]=0; command(g_line,now); }
            g_line_len=0; g_line_overflow=false;
        } else if (g_line_len < sizeof(g_line)-1) g_line[g_line_len++]=c;
        else g_line_overflow=true;
    }
}
void setup(void) {
    for (int i = 0; i < 2; ++i) {
        digitalWrite(kServoPin[i], LOW); pinMode(kServoPin[i], OUTPUT);
        g_direction[i]=kServoDirectionDefault[i]; g_trim[i]=kServoTrimDefaultUs[i];
    }
    for (int i = 0; i < 3; ++i) g_axes[i]=kFootAxisDefault[i];
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);
    getup_init(&g_state); attitude_reset(&g_attitude);
    Serial.begin(115200); // Never wait for a host: standalone battery boot works too.
    Serial.println("BOOT xiao_getup auto-v5 original-front-reference waiting for healthy IMU");
    g_imu_ok=imu_begin();
    g_last_control=g_last_retry=millis();
    g_last_sample=g_last_control;
    Serial.println(g_imu_ok ? "IMU configured" : "IMU init failed; retrying with outputs OFF");
}
void loop(void) {
    uint32_t now=millis();
    serial_commands(now);
    if (button_edge(now)) {
        if (g_enabled[0] || g_enabled[1] || g_state.auto_getup || g_boot_autostart_pending) { outputs_off(now); Serial.println("OK button off"); }
        else start_motion(now);
    }
    uint32_t elapsed=now-g_last_control;
    if (elapsed<kControlPeriodMs) return;
    g_frame_ms=elapsed; g_last_control=now;
    if (elapsed>100) {
        stop_outputs(now,"CONTROL_OVERRUN"); attitude_reset(&g_attitude); g_sample_ok=false;
        Serial.println("FAULT control overrun; outputs OFF");
    }
    if (!g_imu_ok && now-g_last_retry>=1000) {
        // Outputs are already off on boot and after an IMU fault. Do not
        // cancel the pending first startup merely because init needs retrying.
        g_last_retry=now;
        g_imu_ok=imu_begin();
        attitude_reset(&g_attitude); g_sample_ok=false;
        g_last_control=millis();
        g_last_sample=g_last_control;
        now=g_last_control;
    }
    int result=g_imu_ok ? imu_sample() : 0;
    if (result==1) {
        float dt=g_sample_ok ? (now-g_last_sample)*0.001f : 0.02f;
        bool updated=attitude_update(&g_attitude,g_acc,g_gyro,dt);
        if (updated) {
            g_last_sample=now; g_sample_ok=true;
            attitude_euler(&g_attitude,&g_roll,&g_pitch);
        } else { attitude_reset(&g_attitude); g_sample_ok=false; }
    }
    if (result<0 || (g_sample_ok && now-g_last_sample>60) ||
        (g_imu_ok && !g_sample_ok && now-g_last_sample>250)) {
        ++g_imu_errors; stop_outputs(now,"IMU_LOST"); g_imu_ok=false; g_sample_ok=false;
        attitude_reset(&g_attitude);
        Serial.println("FAULT IMU lost; outputs OFF");
    }
    if (g_boot_autostart_pending && imu_healthy(now)) {
        g_boot_autostart_pending = false;
        g_state.auto_getup = true;
        g_state.ended = now;
        g_enabled[0] = g_enabled[1] = true;
        g_stop_reason = "NONE";
        Serial.println("OK automatic recovery ON; monitoring falls (button/off stops)");
    }
    if (g_state.auto_getup && !imu_healthy(now)) stop_outputs(now,"IMU_NOT_FRESH");
    bool was_running=g_state.mode==GETUP_RUNNING;
    if (imu_healthy(now) && getup_poll(&g_state,g_roll,g_pitch,now)) {
        if (g_state.centering) getup_center_step(&g_state);
        else if (getup_reference_step(&g_state,now)) { /* Existing teacher path. */ }
        else {
            float obs[OBS_DIM]={g_roll,g_pitch,g_state.servo1,g_state.servo2};
            float action[ACTION_DIM]; forward_policy(obs,action);
            if (!isfinite(action[0]) || !isfinite(action[1])) {
                stop_outputs(now,"INVALID_POLICY");
                Serial.println("FAULT invalid policy; outputs OFF");
            }
            else getup_apply_action(&g_state,action[0],action[1]);
        }
    }
    if (was_running && g_state.mode!=GETUP_RUNNING) {
        if (!g_state.auto_getup && (g_enabled[0] || g_enabled[1])) {
            stop_outputs(now,"GETUP_TIMEOUT");
            Serial.println("FAULT getup timeout; outputs OFF");
        } else if (g_state.auto_getup) Serial.println("DONE foot level and targets near center (no angle feedback)");
    }
    if (g_probe_channel>=0) {
        if ((int32_t)(now-g_probe_until)>=0) { stop_outputs(now,"PROBE_FINISHED"); Serial.println("OK probe finished; outputs OFF"); }
        else {
            int ch=g_probe_channel;
            int target=servo_pulse(ch,g_probe_rad);
            int delta=target-g_pulse[ch];
            g_pulse[ch]+=delta>20 ? 20 : delta < -20 ? -20 : delta;
        }
    } else if (g_enabled[0] && g_enabled[1]) {
        g_pulse[0]=servo_pulse(0,g_state.servo1);
        g_pulse[1]=servo_pulse(1,g_state.servo2);
    }
    pulse_servos();
    digitalWrite(LED_BUILTIN,g_state.mode==GETUP_RUNNING ? LOW : HIGH);
    if (g_report && now-g_last_report>=400) { g_last_report=now; report_status(now); }
}
