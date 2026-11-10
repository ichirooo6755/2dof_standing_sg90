"""Direction-commanded motion for the two-servo self-righting robot.

The servos are not parallel: servo 1 turns about the foot X axis and servo 2
about the foot Y axis. Commands are in that body frame, not the world frame,
because the IMU has no magnetometer.

The step that actually translates this linkage tips past horizontal and stands
back up. The command is the body direction at the moment it is given, held
fixed for the episode. The actor never sees the true velocity. That is
appended for the critic only.
"""

import math
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from robo1_env import FALLEN_POSES, roll_pitch_to_quat


COMMANDS = {
    "forward": np.array([1.0, 0.0], dtype=np.float64),
    "back": np.array([-1.0, 0.0], dtype=np.float64),
    "left": np.array([0.0, 1.0], dtype=np.float64),
    "right": np.array([0.0, -1.0], dtype=np.float64),
}
COMMAND_NAMES = tuple(COMMANDS)
ACTOR_DIM = 12  # gravity, gyro, previous action, commanded angles, command
TARGET_SPEED = 0.06  # m/s along the command


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


class WalkEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, xml_path="robo1.xml"):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(str(Path(xml_path)))
        self.data = mujoco.MjData(self.model)
        self.nominal_mass = self.model.body_mass.copy()
        self.nominal_inertia = self.model.body_inertia.copy()
        self.nominal_gain = self.model.actuator_gainprm[:, 0].copy()
        self.nominal_bias = self.model.actuator_biasprm[:, 1].copy()

        self.frame_skip = 20
        self.max_steps = 400
        self.target_delta = 0.08
        self.target_limit = 1.55
        self.settle_steps = 160

        self.foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "foot")
        s1 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo1_joint")
        s2 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo2_joint")
        self.s1_q = self.model.jnt_qposadr[s1]
        self.s2_q = self.model.jnt_qposadr[s2]

        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        high = np.ones(ACTOR_DIM + 3, dtype=np.float32) * 8.0
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.target = np.zeros(2, dtype=np.float64)
        self.prev_action = np.zeros(2, dtype=np.float64)
        self.command = COMMANDS["forward"].copy()
        self.command_name = "forward"
        self.step_count = 0
        self.start_pos = np.zeros(3)
        self.start_rot = np.eye(3)
        self.queue = []
        self.upright = 1.0
        self.prev_progress = 0.0
        self.grav_noise = 0.0
        self.gyro_noise = 0.0

    def _rotation(self):
        return self.data.xmat[self.foot_id].reshape(3, 3).copy()

    def _body_velocity(self):
        velocity = self._rotation().T @ self.data.qvel[:3]
        return velocity.astype(np.float64)

    def _gravity_gyro(self):
        rotation = self._rotation()
        gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        gyro = self.data.qvel[3:6].copy()
        return gravity, gyro

    def _obs(self):
        gravity, gyro = self._gravity_gyro()
        if self.grav_noise:
            gravity = gravity + self.np_random.normal(0.0, self.grav_noise, size=3)
        if self.gyro_noise:
            gyro = gyro + self.np_random.normal(0.0, self.gyro_noise, size=3)
        velocity = self._body_velocity()
        actor = np.concatenate(
            [gravity, gyro, self.prev_action, self.target, self.command]
        )
        return np.concatenate([actor, velocity]).astype(np.float32)

    def reward(self, action, speed, upright):
        # Clip only past speeds this linkage does not hold. A tight clip
        # cancels the forward and backward kicks of a tip-and-stand step.
        tracking = float(np.clip(speed, -2.0, 2.0) / TARGET_SPEED)
        smooth = float(np.sum((action - self.prev_action) ** 2))
        energy = float(np.sum(self.data.actuator_force ** 2))
        down = 1.0 if upright < 0.0 else 0.0
        return float(tracking - 0.25 * down - 0.05 * smooth - 1.0 * energy)

    def _nominal(self):
        self.model.body_mass[:] = self.nominal_mass
        self.model.body_inertia[:] = self.nominal_inertia
        self.model.actuator_gainprm[:, 0] = self.nominal_gain
        self.model.actuator_biasprm[:, 1] = self.nominal_bias
        self.model.actuator_forcerange[:] = [-0.15, 0.15]
        self.model.geom_friction[:, 0] = 1.0
        self.queue.clear()
        self.grav_noise = 0.0
        self.gyro_noise = 0.0

    def _randomize_domain(self):
        scale = float(self.np_random.uniform(0.85, 1.15))
        self.model.body_mass[:] = self.nominal_mass * scale
        self.model.body_inertia[:] = self.nominal_inertia * scale
        gain = float(self.np_random.uniform(0.6, 1.4))
        self.model.actuator_gainprm[:, 0] = self.nominal_gain * gain
        self.model.actuator_biasprm[:, 1] = self.nominal_bias * gain
        # 0.15 N·m is the SG90 near 5 V. The low end stands in for a sagging 1S pack.
        torque = float(self.np_random.uniform(0.08, 0.16))
        self.model.actuator_forcerange[:] = [-torque, torque]
        self.model.geom_friction[:, 0] = float(self.np_random.uniform(0.4, 1.8))
        delay = int(self.np_random.integers(0, 3))
        self.queue = [np.zeros(2) for _ in range(delay)]
        self.grav_noise = float(self.np_random.uniform(0.0, 0.03))
        self.gyro_noise = float(self.np_random.uniform(0.0, 0.08))

    def _pose(self, fallen, yaw=None, pose=None):
        if yaw is None:
            yaw = float(self.np_random.uniform(0.0, 2.0 * math.pi))
        yaw_q = np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])
        if not fallen:
            tilt = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            if pose is None:
                pose = self.np_random.choice(tuple(FALLEN_POSES))
            tilt = roll_pitch_to_quat(*FALLEN_POSES[pose])
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.qpos[2] = 0.08
        self.data.qpos[3:7] = quat_mul(yaw_q, tilt)
        self.target[:] = 0.0
        self.prev_action[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        for _ in range(self.settle_steps):
            mujoco.mj_step(self.model, self.data)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        options = {} if options is None else options
        name = options.get("command")
        if name is None:
            name = self.np_random.choice(COMMAND_NAMES)
        self.command_name = str(name)
        self.command = COMMANDS[self.command_name].copy()
        fallen = bool(options["fallen"]) if "fallen" in options else bool(self.np_random.random() < 0.4)
        if options.get("randomize") is False:
            self._nominal()
        else:
            self._randomize_domain()
        self._pose(fallen, yaw=options.get("yaw"), pose=options.get("pose"))
        self.step_count = 0
        self.prev_progress = 0.0
        self.start_pos = self.data.qpos[:3].copy()
        self.start_rot = self._rotation()
        gravity, _ = self._gravity_gyro()
        self.upright = float(-gravity[2])
        return self._obs(), {"command": self.command_name, "fallen_start": fallen}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self.queue.append(action.copy())
        applied = self.queue.pop(0)
        self.target += applied * self.target_delta
        self.target = np.clip(self.target, -self.target_limit, self.target_limit)
        for _ in range(self.frame_skip):
            self.data.ctrl[:] = self.target
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        velocity = self._body_velocity()
        gravity, _ = self._gravity_gyro()
        upright = float(-gravity[2])
        displacement = self.start_rot.T @ (self.data.qpos[:3] - self.start_pos)
        aligned = float(displacement[0] * self.command[0] + displacement[1] * self.command[1])
        dt = self.model.opt.timestep * self.frame_skip
        speed = (aligned - self.prev_progress) / dt
        self.prev_progress = aligned
        reward = self.reward(applied, speed, upright)
        self.upright = upright
        self.prev_action = applied.copy()

        terminated = bool(self.data.qpos[2] < -0.3 or not np.isfinite(self.data.qpos).all())
        truncated = self.step_count >= self.max_steps
        info = {
            "command": self.command_name,
            "aligned_displacement": aligned,
            "upright": upright,
            "vx": float(velocity[0]),
            "vy": float(velocity[1]),
        }
        return self._obs(), reward, terminated, truncated, info


def _demo():
    env = WalkEnv()
    obs, info = env.reset(options={"command": "forward", "fallen": False, "randomize": False})
    assert obs.shape == (ACTOR_DIM + 3,)
    assert abs(float(np.linalg.norm(obs[10:12])) - 1.0) < 1e-5
    assert obs.shape[0] == 15
    forward = env.reward(np.zeros(2), 0.06, 1.0)
    still = env.reward(np.zeros(2), 0.0, 1.0)
    wrong = env.reward(np.zeros(2), -0.06, 1.0)
    assert forward > still > wrong
    obs, info = env.reset(options={"command": "left", "fallen": True, "randomize": False})
    assert info["fallen_start"]
    obs, reward, _, _, info = env.step(np.zeros(2))
    assert np.isfinite(reward)
    low, high = env.model.actuator_forcerange[0]
    assert abs(low + 0.15) < 1e-6 and abs(high - 0.15) < 1e-6
    print("walk env ok", "upright", round(info["upright"], 3))


if __name__ == "__main__":
    _demo()
