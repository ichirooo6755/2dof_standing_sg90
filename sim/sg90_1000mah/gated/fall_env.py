"""Recovery from moving falls, with a sampled stationary-start gate.

The candidate gate is evaluated in simulation. The normal SG90 recovery
actor, IMU filter, actuator and retry supervisor are unchanged.
"""
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from env import SG90Env, FilteredEnv
from model_geometry import geom_vertices_world, place_neutral_robot


DIRECTIONS = {
    "usb": -90,
    "usb_left": -135,
    "left": 180,
    "opposite_left": 135,
    "opposite": 90,
    "opposite_right": 45,
    "right": 0,
    "usb_right": -45,
}
FALL_UP = np.cos(np.deg2rad(45))
UPRIGHT_UP = np.cos(np.deg2rad(25))
STILL_DIRECTION_DOT = np.cos(np.deg2rad(3))


class StillnessGate:
    def __init__(self, wait_s, filter_s):
        self.wait_s = wait_s
        self.filter_s = filter_s
        self.detected_at = None
        self.still_since = None
        self.ready_at = None
        self.last_sample = None
        self.accel_average = None
        self.gyro_average = None
        self.quiet_down = None
        self.invalid_samples = 0
        self.max_quiet_s = 0.

    def update(self, time_s, down, gyro, accel, valid):
        if not valid:
            self.invalid_samples += 1
            self.still_since = None
            self.ready_at = None
            self.accel_average = None
            return
        if self.accel_average is None or not self.filter_s:
            self.accel_average = accel.copy()
            self.gyro_average = gyro.copy()
        else:
            alpha = -np.expm1(-(time_s - self.last_sample) / self.filter_s)
            self.accel_average += alpha * (accel - self.accel_average)
            self.gyro_average += alpha * (gyro - self.gyro_average)
        self.last_sample = time_s
        up = -float(down[2])
        if up < FALL_UP and self.detected_at is None:
            self.detected_at = time_s
        if self.detected_at is None:
            return
        # The norm tests apply equally when the IMU is sideways or inverted.
        still = (np.linalg.norm(self.gyro_average) < .30
                 and np.linalg.norm(gyro) < 2.5
                 and abs(np.linalg.norm(self.accel_average) - 1.) < .12)
        fallen = up < UPRIGHT_UP
        if not still or not fallen:
            self.still_since = None
            self.ready_at = None
            return
        if self.still_since is None or (self.filter_s and down @ self.quiet_down < STILL_DIRECTION_DOT):
            self.still_since = time_s
            self.quiet_down = down.copy()
            self.ready_at = None
        quiet_s = time_s - self.still_since
        self.max_quiet_s = max(self.max_quiet_s, quiet_s)
        if quiet_s >= self.wait_s - 1e-9 and self.ready_at is None:
            self.ready_at = time_s


class GatedRecoveryEnv(SG90Env):
    def __init__(self, randomize=True, wait_s=.3, filter_s=.08, physics_dt=.001, max_wait_s=10.):
        super().__init__(randomize=randomize)
        self.model.opt.timestep = self.physics_dt = physics_dt
        self.frame_skip = round(.02 / physics_dt)
        self.wait_s = wait_s
        self.filter_s = filter_s
        self.wait_limit_steps = round(max_wait_s / .02)
        self.gate = None
        self.release_at = None
        self.motors_enabled_at = None
        self.first_change_at = None
        update = self.runtime.update

        def observe(accel, gyro, dt, *args, **kwargs):
            valid = update(accel, gyro, dt, *args, **kwargs)
            if self.gate is not None and self.motors_enabled_at is None:
                self.gate.update(self.data.time - self.release_at, self.runtime.down,
                                 self.runtime.gyro, accel, valid)
            return valid

        self.runtime.update = observe
        self.colliders = [
            i for i in range(self.model.ngeom)
            if self.model.geom_bodyid[i] and self.model.geom_contype[i]
        ]

    def reset(self, seed=None, options=None):
        self.gate = None
        self.motors_enabled_at = None
        self.first_change_at = None
        super().reset(seed=seed, options={"tilt": 0., "azimuth": 0.})
        options = options or {}
        rng = self.np_random
        azimuth = np.deg2rad(options.get("axis_deg", rng.uniform(-180, 180)))
        tilt_deg = options.get("tilt_deg", rng.choice([65, 85, 105, 135, 160, 180]))
        push = options.get("push_rad_s", rng.uniform(-1., 3.))
        bent = options.get("bent", rng.random() < .45)
        clearance = options.get("clearance_m", rng.uniform(.001, .018))
        theta = np.deg2rad(tilt_deg)
        place_neutral_robot(self.model, self.data, theta, azimuth, clearance=0,
                            support_points=self.neutral_support)
        self.data.qpos[-2:] = rng.uniform(-1.15, 1.15, 2) if bent else 0.
        mujoco.mj_forward(self.model, self.data)
        lowest = min(geom_vertices_world(self.model, self.data, i)[:, 2].min()
                     for i in self.colliders)
        self.data.qpos[2] += clearance - lowest
        self.data.qvel[:] = 0.
        self.command[:] = self.data.qpos[-2:]
        self.servo_target[:] = self.command
        self.previous[:] = 0.
        self.queue = [self.command.copy() for _ in range(self.delay)]
        self.strategy.reset(seed)
        self.runtime.reset()
        self.control_active = False
        self.stop_requested = False
        self.next_sample = self.data.time
        self.sample_time = None
        mujoco.mj_forward(self.model, self.data)
        FilteredEnv.before_settle(self)
        self.release_at = self.data.time
        self.gate = StillnessGate(float(options.get("wait_s", self.wait_s)), self.filter_s)
        self.data.qvel[3:5] = push * np.array([np.cos(azimuth), np.sin(azimuth)])
        self.data.qvel[2] = -float(options.get("drop_m_s", rng.uniform(0., .12)))
        self.data.ctrl[:] = 0.
        self.no_trigger_reason = None
        self.case = {
            "tilt_deg": int(tilt_deg), "axis_deg": float(np.rad2deg(azimuth)),
            "push_rad_s": float(push), "bent": bool(bent), "wait_s": self.gate.wait_s,
        }

        # A held PWM target is modeled during the wait. No new commands
        # are emitted until an entire interval of quiet samples has passed.
        for _ in range(self.wait_limit_steps):
            self._physics(self.command)
            if any(w.number for w in self.data.warning):
                self.no_trigger_reason = "physics_fault"
                break
            if self.gate.ready_at is not None:
                self.motors_enabled_at = self.data.time - self.release_at
                self.control_active = True
                break
        if self.motors_enabled_at is None and self.no_trigger_reason is None:
            settled_up = -float(self.runtime.down[2])
            self.no_trigger_reason = (
                "filter_not_ready" if not self.runtime.ready
                else "settled_upright" if settled_up > UPRIGHT_UP
                else "below_fall_threshold" if self.gate.detected_at is None
                else "stillness_timeout"
            )
        return self._obs(), self.timing()

    def timing(self):
        return {
            **self.case,
            "fall_detected_s": self.gate.detected_at,
            "quiet_since_s": self.gate.still_since,
            "max_quiet_s": self.gate.max_quiet_s,
            "invalid_imu_samples": self.gate.invalid_samples,
            "gate_ready_s": self.gate.ready_at,
            "policy_start_s": self.motors_enabled_at,
            "first_command_change_s": self.first_change_at,
            "no_trigger_reason": self.no_trigger_reason,
        }

    def step(self, action):
        if self.motors_enabled_at is None:
            benign = self.no_trigger_reason == "settled_upright"
            return self._obs(), 0. if benign else -30., True, False, {
                **self.timing(), "is_success": False,
                "control_fault": self.no_trigger_reason in ("physics_fault", "filter_not_ready"),
            }
        previous_command = self.command.copy()
        command_time = self.data.time - self.release_at
        obs, reward, terminated, truncated, info = super().step(action)
        if self.first_change_at is None and np.max(np.abs(previous_command - self.command)) > 1e-6:
            self.first_change_at = command_time
        return obs, reward, terminated, truncated, {**info, **self.timing()}
