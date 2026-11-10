"""Train commanded motion without forgetting how to stand up.

The actor sees only what the nRF54L15 can know: gravity, gyro, the previous
action, the angles it commanded, and the body-frame direction. The critic also
sees the true body velocity.
"""

import argparse

import numpy as np
import torch as th
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.vec_env import DummyVecEnv

from walk_env import ACTOR_DIM, COMMAND_NAMES, WalkEnv


class ActorSlice(BaseFeaturesExtractor):
    def __init__(self, observation_space: spaces.Box, actor_dim: int = ACTOR_DIM):
        super().__init__(observation_space, features_dim=actor_dim)
        self.actor_dim = actor_dim

    def forward(self, observations: th.Tensor) -> th.Tensor:
        return observations[..., : self.actor_dim]


class SmallAsymmetricPolicy(ActorCriticPolicy):
    """32-unit actor. The critic reads the privileged velocity as well."""

    def __init__(self, observation_space, action_space, lr_schedule, actor_dim=ACTOR_DIM, **kwargs):
        self.actor_dim = actor_dim
        kwargs["net_arch"] = dict(pi=[32, 32], vf=[32, 32])
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
            th.nn.Linear(full, 32),
            th.nn.Tanh(),
            th.nn.Linear(32, 32),
            th.nn.Tanh(),
            th.nn.Linear(32, 1),
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


def evaluate(model):
    motion = []
    for name in COMMAND_NAMES:
        env = WalkEnv("robo1.xml")
        obs, _ = env.reset(options={"command": name, "fallen": False})
        done = False
        info = {}
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
        motion.append((name, info["aligned_displacement"], info["upright"]))
    recovery = []
    for name in COMMAND_NAMES:
        env = WalkEnv("robo1.xml")
        obs, _ = env.reset(options={"command": name, "fallen": True})
        done = False
        info = {}
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
        recovery.append(info["upright"])
    return motion, float(np.mean(recovery))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=300_000)
    parser.add_argument("--chunk", type=int, default=50_000)
    parser.add_argument("--model-out", default="robo1_walk_ppo")
    args = parser.parse_args()

    env = DummyVecEnv([make_env])
    model = PPO(
        SmallAsymmetricPolicy,
        env,
        n_steps=1024,
        batch_size=256,
        learning_rate=3e-4,
        gamma=0.99,
        ent_coef=0.005,
        verbose=0,
        device="cpu",
        seed=0,
    )
    best = -1e9
    trained = 0
    while trained < args.timesteps:
        step = min(args.chunk, args.timesteps - trained)
        model.learn(total_timesteps=step, reset_num_timesteps=False)
        trained += step
        motion, upright = evaluate(model)
        distances = [distance for _, distance, _ in motion]
        mean = float(np.mean(distances))
        summary = " ".join(
            f"{name}={distance:+.3f}/{up:.2f}" for name, distance, up in motion
        )
        print(
            f"steps {trained} mean {mean:+.3f} recover {upright:.2f} {summary}",
            flush=True,
        )
        # Prefer moving the right way without staying down.
        score = mean + 0.05 * upright
        if score > best and upright > 0.4:
            best = score
            model.save(args.model_out)
            print(f"saved {args.model_out}", flush=True)


if __name__ == "__main__":
    main()
