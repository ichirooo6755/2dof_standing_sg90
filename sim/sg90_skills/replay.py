"""Paired MuJoCo evaluations and videos for the learned SG90 policies."""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from envs import EdgeEnv, MoveEnv
from policies import ResidualPolicy, SeedPolicy


class NeutralPolicy:
    def predict(self, obs, deterministic=True):
        return np.zeros(2, dtype=np.float32), None


def load_policy(task, path):
    if path is None:
        return NeutralPolicy()
    config = json.loads(path.read_text())
    if config["task"] != task:
        raise ValueError(f"{path} contains a {config['task']} policy")
    if "residual_model" not in config:
        return SeedPolicy(task, config["parameters"])
    from stable_baselines3 import PPO
    import torch

    torch.set_num_threads(1)
    seed = SeedPolicy(task, config["seed_parameters"])
    model = PPO.load(path.parent / config["residual_model"], device="cpu")
    return ResidualPolicy(seed, model, config["residual_scale"])


class Rollout:
    def __init__(self, env, start, policy, seed):
        self.env = env
        self.policy = policy
        self.obs, _ = env.reset(seed=seed, options={"start": start})
        self.seed = seed
        self.info = {}
        self.max_tilt = abs(np.rad2deg(env.initial_pitch))
        self.done = False

    def step(self):
        if self.done:
            return
        action, _ = self.policy.predict(self.obs, deterministic=True)
        self.obs, _, terminal, truncated, self.info = self.env.step(action)
        tilt = np.rad2deg(np.arccos(np.clip(self.info["base_up"], -1, 1)))
        self.max_tilt = max(self.max_tilt, tilt)
        self.done = terminal or truncated

    def metrics(self):
        return {
            "seed": self.seed,
            "seconds": self.env.steps * 0.02,
            "max_tilt_deg": float(self.max_tilt),
            **self.info,
        }


def evaluate(env, start, policy, seeds):
    rows = []
    for seed in seeds:
        rollout = Rollout(env, start, policy, seed)
        while not rollout.done:
            rollout.step()
        rows.append(rollout.metrics())
    summary = {
        "episodes": len(rows),
        "successes": sum(row["is_success"] for row in rows),
        "faults": sum(row["control_fault"] for row in rows),
        "mean_progress_m": float(np.mean([row["progress_m"] for row in rows])),
        "mean_abs_lateral_m": float(np.mean([abs(row["lateral_m"]) for row in rows])),
        "mean_max_tilt_deg": float(np.mean([row["max_tilt_deg"] for row in rows])),
        "max_tilt_deg": max(row["max_tilt_deg"] for row in rows),
        "mean_edge_seconds": float(np.mean([row["edge_seconds"] for row in rows])),
        "max_edge_seconds": max(row["edge_seconds"] for row in rows),
        "trials": rows,
    }
    if env.task == "move":
        summary["mean_second_half_progress_m"] = float(
            np.mean([row["second_half_progress_m"] for row in rows])
        )
    return summary


def record(task, env_type, start, policy, seed, path):
    import imageio.v2 as imageio
    import matplotlib
    from PIL import Image, ImageDraw, ImageFont

    envs = [env_type(randomize=True, start=start, difficulty=1.) for _ in range(2)]
    renderers = []
    font_path = Path(matplotlib.get_data_path()) / "fonts/ttf/DejaVuSans.ttf"
    font = ImageFont.truetype(str(font_path), 16)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [-0.025, 0, 0.045]
    camera.distance = 0.43
    camera.azimuth = 140
    camera.elevation = -20
    try:
        rollouts = [
            Rollout(envs[0], start, policy, seed),
            Rollout(envs[1], start, NeutralPolicy(), seed),
        ]
        for env in envs:
            renderers.append(mujoco.Renderer(env.model, height=384, width=480))
        with imageio.get_writer(path, fps=25, codec="libx264", quality=7) as writer:
            for frame in range(envs[0].horizon // 2 + 1):
                panels = []
                for rollout, renderer, label in zip(
                    rollouts, renderers, ("Learned policy", "Neutral commands")
                ):
                    if frame:
                        rollout.step()
                        rollout.step()
                    renderer.update_scene(rollout.env.data, camera=camera)
                    panel = Image.fromarray(renderer.render())
                    draw = ImageDraw.Draw(panel)
                    draw.rectangle((0, 0, 480, 73), fill=(22, 27, 34))
                    stats = rollout.metrics()
                    end = " | episode ended" if rollout.done else ""
                    if task == "move":
                        metric = f"Forward {100 * stats.get('progress_m', 0):+.2f} cm"
                        metric += f"  |  lateral {100 * stats.get('lateral_m', 0):+.2f} cm"
                    else:
                        metric = f"Longest stable edge support {stats.get('edge_seconds', 0):.2f} s"
                    draw.text((10, 5), f"{label}  |  {stats['seconds']:.2f} s{end}", font=font, fill="white")
                    draw.text((10, 28), metric, font=font, fill=(160, 218, 255))
                    draw.text((10, 51), f"MuJoCo  |  {start} start  |  seed {seed}", font=font, fill=(195, 200, 208))
                    panels.append(np.asarray(panel))
                combined = np.concatenate(panels, axis=1)
                writer.append_data(combined)
                if all(rollout.done for rollout in rollouts):
                    for _ in range(19):
                        writer.append_data(combined)
                    break
        return [rollout.metrics() for rollout in rollouts]
    finally:
        for renderer in renderers:
            renderer.close()
        for env in envs:
            env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", choices=["move", "edge"])
    parser.add_argument("policy", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--start", choices=["flat", "near", "side"], default="flat")
    parser.add_argument("--first-seed", type=int, default=96000)
    parser.add_argument("--episodes", type=int, default=24)
    parser.add_argument("--video-seed", type=int)
    parser.add_argument("--physics-timestep", type=float, default=.001)
    args = parser.parse_args()
    step_ratio = .02 / args.physics_timestep if args.physics_timestep > 0 else 0.
    if not np.isfinite(step_ratio) or step_ratio < 1 or not np.isclose(step_ratio, round(step_ratio)):
        parser.error("physics timestep must divide the 20 ms control cycle")
    if args.video_seed is not None and args.physics_timestep != .001:
        parser.error("videos use the original 1 ms physics timestep")
    args.out.mkdir(parents=True, exist_ok=True)
    env_type = MoveEnv if args.task == "move" else EdgeEnv
    policy = load_policy(args.task, args.policy)
    env = env_type(randomize=True, start=args.start, difficulty=1.)
    env.model.opt.timestep = env.physics_dt = args.physics_timestep
    env.frame_skip = round(.02 / env.physics_dt)
    seeds = range(args.first_seed, args.first_seed + args.episodes)
    try:
        learned = evaluate(env, args.start, policy, seeds)
        neutral = evaluate(env, args.start, NeutralPolicy(), seeds)
    finally:
        env.close()
    result = {
        "task": args.task,
        "start": args.start,
        "policy": str(args.policy),
        "physics_timestep_s": args.physics_timestep,
        "learned": learned,
        "neutral": neutral,
    }
    path = args.out / "paired_evaluation.json"
    path.write_text(json.dumps(result, indent=2) + "\n")
    if args.video_seed is not None:
        result["video_rollouts"] = record(
            args.task, env_type, args.start, policy, args.video_seed, args.out / "comparison.mp4"
        )
        path.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({name: {k: v for k, v in result[name].items() if k != "trials"}
                      for name in ("learned", "neutral")}), flush=True)


if __name__ == "__main__":
    main()
