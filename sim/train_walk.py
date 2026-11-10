"""Train a policy that walks forward, back, left, or right on command.

An open-loop swing that actually moves the body is cloned first. PPO then
lengthens that gait and cuts down the sideways drift.
"""

import argparse
import math

import mujoco
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

from walk_env import COMMAND_NAMES, WalkEnv


# servo1 amplitude, servo2 amplitude, angular frequency, phase.
# Found by searching periodic swings on this model. Each one moves the foot
# along its named axis.
GAITS = {
    # Amplitudes stay inside the 4 rad/s command limit, so the policy can copy them.
    "forward": (1.386, -0.943, 2.589, 1.283),
    "back": (1.458, -1.169, 2.606, -1.748),
    "left": (1.457, -1.129, 2.607, 2.236),
    "right": (-1.460, 1.221, 2.564, -2.067),
}


def make_env():
    return WalkEnv("robo1.xml")


def collect_gait(yaw_count=4):
    observations = []
    actions = []
    for name, (a1, a2, omega, phase) in GAITS.items():
        for yaw_index in range(yaw_count):
            env = WalkEnv("robo1.xml")
            env.reset(seed=yaw_index, options={"command": name})
            # reset() randomizes yaw. Replay from a fixed yaw so every direction
            # is seen at the same headings.
            yaw = yaw_index * (math.pi / 2.0)
            env.data.qpos[:] = 0.0
            env.data.qvel[:] = 0.0
            env.data.qpos[2] = 0.08
            env.data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
            env.target[:] = 0.0
            env.data.ctrl[:] = 0.0
            mujoco.mj_forward(env.model, env.data)
            for _ in range(env.settle_steps):
                mujoco.mj_step(env.model, env.data)
            env.start_pos = env.data.qpos[:3].copy()
            env.start_rot = env._rotation()
            for i in range(env.max_steps):
                t = i * 0.02
                desired = np.array(
                    [a1 * math.sin(omega * t), a2 * math.sin(omega * t + phase)],
                    dtype=np.float64,
                )
                action = np.clip((desired - env.target) / env.target_delta, -1.0, 1.0)
                observations.append(env._obs())
                actions.append(action.astype(np.float32))
                _, _, terminated, truncated, _ = env.step(action)
                if terminated or truncated:
                    break
    return np.asarray(observations, dtype=np.float32), np.asarray(actions, dtype=np.float32)


def clone_gaits(model, observations, actions):
    obs_tensor = torch.as_tensor(observations, dtype=torch.float32, device=model.device)
    act_tensor = torch.as_tensor(actions, dtype=torch.float32, device=model.device)
    optimizer = torch.optim.Adam(model.policy.parameters(), lr=1e-3)
    for epoch in range(2000):
        mean = model.policy.get_distribution(obs_tensor).distribution.mean
        loss = torch.mean((mean - act_tensor) ** 2)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        if epoch % 500 == 0:
            print("clone", epoch, "loss", round(float(loss.detach().cpu()), 5), flush=True)


def evaluate(model, episodes=2):
    rows = []
    for name in COMMAND_NAMES:
        distances = []
        for _ in range(episodes):
            env = WalkEnv("robo1.xml")
            obs, _ = env.reset(options={"command": name})
            done = False
            info = {}
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = env.step(action)
                done = terminated or truncated
            distances.append(info["aligned_displacement"])
        rows.append((name, float(np.mean(distances))))
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=600_000)
    parser.add_argument("--chunk", type=int, default=100_000)
    parser.add_argument("--model-out", default="robo1_walk_ppo")
    args = parser.parse_args()

    observations, actions = collect_gait()
    print("gait samples", len(observations), flush=True)
    env = DummyVecEnv([make_env])
    model = PPO(
        "MlpPolicy",
        env,
        n_steps=1024,
        batch_size=256,
        learning_rate=3e-4,
        gamma=0.99,
        ent_coef=0.01,
        verbose=0,
        device="cpu",
        seed=0,
    )
    clone_gaits(model, observations, actions)
    rows = evaluate(model)
    best = float(np.mean([distance for _, distance in rows]))
    print("cloned", " ".join(f"{name}={distance:+.3f}" for name, distance in rows), "mean", f"{best:+.3f}", flush=True)
    model.save(args.model_out)
    model.learning_rate = 1e-4
    model.lr_schedule = lambda _progress: 1e-4
    model.ent_coef = 0.001
    trained = 0
    print("command aligned_displacement_m", flush=True)
    while trained < args.timesteps:
        step = min(args.chunk, args.timesteps - trained)
        model.learn(total_timesteps=step, reset_num_timesteps=False)
        trained += step
        rows = evaluate(model)
        mean = float(np.mean([distance for _, distance in rows]))
        summary = " ".join(f"{name}={distance:+.3f}" for name, distance in rows)
        print(f"steps {trained} mean {mean:+.3f} {summary}", flush=True)
        if mean > best:
            best = mean
            model.save(args.model_out)
            print(f"saved {args.model_out} mean {best:+.3f}", flush=True)

    saved = PPO.load(args.model_out, env=env, device="cpu")
    rows = evaluate(saved)
    mean = float(np.mean([distance for _, distance in rows]))
    print("kept", " ".join(f"{name}={distance:+.3f}" for name, distance in rows), "mean", f"{mean:+.3f}", flush=True)
    if mean <= 0.0:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
