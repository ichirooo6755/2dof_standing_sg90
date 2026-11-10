"""Commanded walking for the two-servo robot.

Forward/back is the foot's +X/-X, left/right is +Y/-Y, both in the yaw frame
of the foot. Servo 2 swings the arm in the X-Z plane, servo 1 in the Y-Z plane.
"""

import math
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces


COMMANDS = {
    "forward": np.array([1.0, 0.0], dtype=np.float64),
    "back": np.array([-1.0, 0.0], dtype=np.float64),
    "left": np.array([0.0, 1.0], dtype=np.float64),
    "right": np.array([0.0, -1.0], dtype=np.float64),
}
COMMAND_NAMES = tuple(COMMANDS)


def quat_to_roll_pitch(q):
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


class WalkEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, xml_path="robo1.xml"):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(str(Path(xml_path)))
        self.data = mujoco.MjData(self.model)
        self.model.geom_friction[:, 0] = 1.4

        self.frame_skip = 20
        self.max_steps = 400
        self.target_delta = 0.08
        self.target_limit = 1.55
        self.settle_steps = 150

        self.foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "foot")
        s1 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo1_joint")
        s2 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo2_joint")
        self.s1_q = self.model.jnt_qposadr[s1]
        self.s2_q = self.model.jnt_qposadr[s2]
        self.s1_v = self.model.jnt_dofadr[s1]
        self.s2_v = self.model.jnt_dofadr[s2]

        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        high = np.array(
            [math.pi, math.pi, 5, 5, 2, 2, 20, 20, 1, 1],
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.target = np.zeros(2, dtype=np.float64)
        self.command = COMMANDS["forward"].copy()
        self.command_name = "forward"
        self.step_count = 0
        self.start_pos = np.zeros(3)
        self.start_rot = np.eye(3)

    def _rotation(self):
        return self.data.xmat[self.foot_id].reshape(3, 3).copy()

    def _body_horizontal_velocity(self):
        rotation = self._rotation()
        velocity = rotation.T @ self.data.qvel[:3]
        return float(velocity[0]), float(velocity[1]), float(self.data.qvel[2])

    def _obs(self):
        roll, pitch = quat_to_roll_pitch(self.data.qpos[3:7])
        vx, vy, _ = self._body_horizontal_velocity()
        return np.array(
            [
                roll,
                pitch,
                vx,
                vy,
                self.data.qpos[self.s1_q],
                self.data.qpos[self.s2_q],
                self.data.qvel[self.s1_v],
                self.data.qvel[self.s2_v],
                self.command[0],
                self.command[1],
            ],
            dtype=np.float32,
        )

    def reward(self, action, vx, vy, vz, roll, pitch):
        progress = vx * self.command[0] + vy * self.command[1]
        lateral = -vx * self.command[1] + vy * self.command[0]
        # Leaning and hopping are the gait. Penalizing tilt made the policy
        # stand still, which never reaches the commanded direction.
        return float(
            8.0 * progress
            - 2.0 * lateral * lateral
            - 0.002 * float(np.dot(action, action))
            - 0.02 * abs(vz)
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        name = None if options is None else options.get("command")
        if name is None:
            name = self.np_random.choice(COMMAND_NAMES)
        self.command_name = str(name)
        self.command = COMMANDS[self.command_name].copy()

        yaw = float(self.np_random.uniform(0.0, 2.0 * math.pi))
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.qpos[2] = 0.08
        self.data.qpos[3:7] = [
            math.cos(yaw / 2.0),
            0.0,
            0.0,
            math.sin(yaw / 2.0),
        ]
        self.target[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        for _ in range(self.settle_steps):
            mujoco.mj_step(self.model, self.data)

        self.step_count = 0
        self.start_pos = self.data.qpos[:3].copy()
        self.start_rot = self._rotation()
        return self._obs(), {"command": self.command_name}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self.target += action * self.target_delta
        self.target = np.clip(self.target, -self.target_limit, self.target_limit)
        for _ in range(self.frame_skip):
            self.data.ctrl[:] = self.target
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        roll, pitch = quat_to_roll_pitch(self.data.qpos[3:7])
        vx, vy, vz = self._body_horizontal_velocity()
        reward = self.reward(action, vx, vy, vz, roll, pitch)

        displacement = self.start_rot.T @ (self.data.qpos[:3] - self.start_pos)
        aligned = float(displacement[0] * self.command[0] + displacement[1] * self.command[1])
        fallen_through = self.data.qpos[2] < -0.25 or not np.isfinite(self.data.qpos).all()
        terminated = bool(fallen_through)
        truncated = self.step_count >= self.max_steps
        info = {
            "command": self.command_name,
            "aligned_displacement": aligned,
            "vx": vx,
            "vy": vy,
        }
        return self._obs(), reward, terminated, truncated, info


def _demo():
    env = WalkEnv()
    for name in COMMAND_NAMES:
        obs, info = env.reset(options={"command": name})
        assert obs.shape == (10,)
        assert info["command"] == name
        assert abs(float(np.linalg.norm(obs[8:10])) - 1.0) < 1e-6
        total = 0.0
        for _ in range(5):
            obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
            total += reward
            assert np.isfinite(reward)
            if terminated or truncated:
                break
        assert np.isfinite(total)
    # Moving with the command is better than moving against it.
    env.command = COMMANDS["forward"].copy()
    forward = env.reward(np.zeros(2), 0.2, 0.0, 0.0, 0.0, 0.0)
    backward = env.reward(np.zeros(2), -0.2, 0.0, 0.0, 0.0, 0.0)
    assert forward > backward
    print("walk env ok")


if __name__ == "__main__":
    _demo()
