"""Train commanded motion without forgetting how to stand up.

The actor sees only what the nRF54L15 can know: gravity, gyro, the previous
action, the angles it commanded, and the body-frame direction. The critic also
sees the true body velocity. 64 units is the smallest width that kept both
the four directions and the get-up swing.
"""

import argparse
import math

import numpy as np
import torch as th
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.vec_env import DummyVecEnv

from getup_reference import getup_sequence_for_pose
from robo1_env import FALLEN_POSES
from walk_env import ACTOR_DIM, COMMAND_NAMES, WalkEnv

# Rate-limited swings that move the body in MuJoCo. Used only to start PPO
# away from the stand-still policy. servo1 amp, servo2 amp, frequency, phase.
GAITS = {
    "forward": (1.386, -0.943, 2.589, 1.283),
    "back": (1.458, -1.169, 2.606, -1.748),
    "left": (1.457, -1.129, 2.607, 2.236),
    "right": (-1.460, 1.221, 2.564, -2.067),
}


class ActorSlice(BaseFeaturesExtractor):
    def __init__(self, observation_space: spaces.Box, actor_dim: int = ACTOR_DIM):
        super().__init__(observation_space, features_dim=actor_dim)
        self.actor_dim = actor_dim

    def forward(self, observations: th.Tensor) -> th.Tensor:
        return observations[..., : self.actor_dim]


class SmallAsymmetricPolicy(ActorCriticPolicy):
    """Two hidden layers. The critic also reads the privileged velocity."""

    def __init__(self, observation_space, action_space, lr_schedule, actor_dim=ACTOR_DIM, hidden=32, **kwargs):
        self.actor_dim = actor_dim
        kwargs["net_arch"] = dict(pi=[hidden, hidden], vf=[hidden, hidden])
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            features_extractor_class=ActorSlice,
            features_extractor_kwargs={"actor_dim": actor_dim},
            **kwargs,
        )
        full = int(np.prod(observation_space.shape))
        self.critic_net = th.nn.Sequential(
            th.nn.Linear(full, hidden),
            th.nn.Tanh(),
            th.nn.Linear(hidden, hidden),
            th.nn.Tanh(),
            th.nn.Linear(hidden, 1),
        )
        self.optimizer = self.optimizer_class(
            self.parameters(), lr=lr_schedule(1), **self.optimizer_kwargs
        )

    def _value(self, obs: th.Tensor) -> th.Tensor:
        return self.critic_net(obs)

    def forward(self, obs: th.Tensor, deterministic: bool = False):
        distribution = self.get_distribution(obs)
        actions = distribution.get_actions(deterministic=deterministic)
        return actions, self._value(obs), distribution.log_prob(actions)

    def evaluate_actions(self, obs: th.Tensor, actions: th.Tensor):
        distribution = self.get_distribution(obs)
        return self._value(obs), distribution.log_prob(actions), distribution.entropy()

    def predict_values(self, obs: th.Tensor) -> th.Tensor:
        return self._value(obs)


def make_env():
    return WalkEnv("robo1.xml")


def gait_action(env, step_index, params):
    a1, a2, omega, phase = params
    t = step_index * 0.02
    desired = np.array([a1 * math.sin(omega * t), a2 * math.sin(omega * t + phase)])
    return np.clip((desired - env.target) / env.target_delta, -1.0, 1.0)


def getup_targets(pose):
    sequence = getup_sequence_for_pose(pose)
    prev = np.zeros(2, dtype=np.float64)
    targets = []
    for waypoint in sequence:
        for i in range(17):
            blend = (i + 1) / 17.0
            targets.append((1.0 - blend) * prev + blend * waypoint)
        prev = waypoint
    return targets


def run_episode(options, expert_at, noise, rng, limit=None):
    env = WalkEnv("robo1.xml")
    env.reset(seed=0, options=options)
    observations = []
    actions = []
    steps = env.max_steps if limit is None else limit
    for i in range(steps):
        expert = np.clip(expert_at(env, i), -1.0, 1.0)
        observations.append(env._obs().copy())
        actions.append(expert.astype(np.float32))
        execute = expert
        if noise:
            execute = np.clip(expert + rng.normal(0.0, noise, size=2), -1.0, 1.0)
        _, _, terminated, truncated, _ = env.step(execute)
        if terminated or truncated:
            break
    return np.asarray(observations, np.float32), np.asarray(actions, np.float32)


def collect_motion():
    """Sine gaits, plus the same gaits with the executed action noised.

    The label stays on the clean sine, so the actor learns to steer back onto
    it. Left is repeated: it is the shortest travel and otherwise drops out.
    """
    rng = np.random.default_rng(0)
    observations = []
    actions = []
    for noise in (0.0, 0.1, 0.25):
        for name, params in GAITS.items():
            copies = 3 if name == "left" else 1
            for yaw_index in range(4):
                for _ in range(copies):
                    obs, act = run_episode(
                        {
                            "command": name,
                            "fallen": False,
                            "randomize": False,
                            "yaw": yaw_index * (math.pi / 2.0),
                        },
                        lambda env, i, params=params: gait_action(env, i, params),
                        noise,
                        rng,
                    )
                    observations.append(obs)
                    actions.append(act)
                    if noise == 0.0 and obs.shape[0] >= 40:
                        observations.append(np.repeat(obs[:40], 6, axis=0))
                        actions.append(np.repeat(act[:40], 6, axis=0))
    return np.concatenate(observations), np.concatenate(actions)


def collect_getup():
    """The get-up swing only. The sine that follows it tips the body again."""
    observations = []
    actions = []
    for name in GAITS:
        for pose in FALLEN_POSES:
            targets = getup_targets(pose)

            def expert_at(env, i, targets=targets):
                return (targets[i] - env.target) / env.target_delta

            obs, act = run_episode(
                {
                    "command": name,
                    "fallen": True,
                    "pose": pose,
                    "randomize": False,
                    "yaw": 0.0,
                },
                expert_at,
                0.0,
                np.random.default_rng(0),
                limit=len(targets),
            )
            observations.append(np.repeat(obs, 6, axis=0))
            actions.append(np.repeat(act, 6, axis=0))
    return np.concatenate(observations), np.concatenate(actions)


def _moved(motion):
    return [distance for _, distance, _, _ in motion]


def fit_actor(model, observations, actions, epochs, lr, every):
    obs = th.as_tensor(observations, device=model.device)
    act = th.as_tensor(actions, device=model.device)
    optimizer = th.optim.Adam(model.policy.parameters(), lr=lr)
    best = None
    for epoch in range(epochs + 1):
        mean = model.policy.get_distribution(obs).distribution.mean
        loss = th.mean((mean - act) ** 2)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        if epoch == 0 or epoch % every != 0:
            continue
        motion, upright = evaluate(model)
        distances = _moved(motion)
        summary = " ".join(
            f"{name}={distance:+.3f}/{up:.2f}" for name, distance, up, _ in motion
        )
        print(f"epoch {epoch} up90 {upright:.2f} {summary}", flush=True)
        # upright is the pose 1.8 s after a fall, not the end of a walk.
        if all(distance > 0.04 for distance in distances) and upright > 0.6:
            score = min(distances) + 0.05 * upright
            if best is None or score > best[0]:
                best = (
                    score,
                    {k: v.detach().cpu().clone() for k, v in model.policy.state_dict().items()},
                    motion,
                    upright,
                )
    return best


def evaluate(model):
    motion = []
    for name in COMMAND_NAMES:
        env = WalkEnv("robo1.xml")
        obs, _ = env.reset(
            seed=0,
            options={"command": name, "fallen": False, "randomize": False, "yaw": 0.0},
        )
        done = False
        info = {}
        total = 0.0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            done = terminated or truncated
        motion.append((name, info["aligned_displacement"], info["upright"], total))
    recovery = []
    for pose in FALLEN_POSES:
        env = WalkEnv("robo1.xml")
        obs, _ = env.reset(
            seed=0,
            options={
                "command": "forward",
                "fallen": True,
                "pose": pose,
                "randomize": False,
                "yaw": 0.0,
            },
        )
        up = 0.0
        for _ in range(90):
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            up = info["upright"]
            if terminated or truncated:
                break
        recovery.append(up)
    return motion, float(np.mean(recovery))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=0)
    parser.add_argument("--chunk", type=int, default=30_000)
    parser.add_argument("--model-out", default="robo1_walk_ppo")
    args = parser.parse_args()

    motion_obs, motion_act = collect_motion()
    getup_obs, getup_act = collect_getup()
    print("samples", len(motion_obs), "getup", len(getup_obs), flush=True)
    env = DummyVecEnv([make_env])
    model = PPO(
        SmallAsymmetricPolicy,
        env,
        policy_kwargs={"hidden": 64},
        n_steps=1024,
        batch_size=256,
        learning_rate=3e-5,
        gamma=0.99,
        ent_coef=0.0,
        clip_range=0.1,
        verbose=0,
        device="cpu",
        seed=0,
    )
    best = fit_actor(
        model,
        np.concatenate([motion_obs, getup_obs]),
        np.concatenate([motion_act, getup_act]),
        epochs=4500,
        lr=1e-3,
        every=500,
    )
    if best is None:
        raise SystemExit("no checkpoint both moved and stood up")
    model.policy.load_state_dict(best[1])
    with th.no_grad():
        model.policy.log_std.fill_(math.log(0.05))
    motion, upright = evaluate(model)
    distances = _moved(motion)
    print(
        "cloned",
        " ".join(f"{name}={distance:+.3f}/{up:.2f}" for name, distance, up, _ in motion),
        f"up90 {upright:.2f}",
        flush=True,
    )
    model.save(args.model_out)
    best_score = float(np.mean(distances)) + 0.05 * upright
    best_params = model.get_parameters()
    trained = 0
    reverts = 0
    while trained < args.timesteps:
        step = min(args.chunk, args.timesteps - trained)
        model.learn(total_timesteps=step, reset_num_timesteps=False)
        trained += step
        motion, upright = evaluate(model)
        distances = _moved(motion)
        mean = float(np.mean(distances))
        summary = " ".join(
            f"{name}={distance:+.3f}/{up:.2f}" for name, distance, up, _ in motion
        )
        print(
            f"steps {trained} mean {mean:+.3f} up90 {upright:.2f} {summary}",
            flush=True,
        )
        score = mean + 0.05 * upright
        if score > best_score and all(distance > 0.04 for distance in distances) and upright > 0.6:
            best_score = score
            best_params = model.get_parameters()
            model.save(args.model_out)
            reverts = 0
            print(f"saved {args.model_out}", flush=True)
        else:
            model.set_parameters(best_params)
            reverts += 1
            print("reverted", flush=True)
            if reverts >= 2:
                print("stop ppo", flush=True)
                break


if __name__ == "__main__":
    main()
