"""Continue PPO recovery training from physical fall releases."""

import os
for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(var, "1")

import argparse
import json
import platform
import time
from collections import Counter
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

from fall_env import DIRECTIONS, GatedRecoveryEnv


HERE = Path(__file__).resolve().parent
INITIAL = HERE.parent / "results/run_20261111/best_trained.zip"


def save(path, data):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)


def fall_cases(held_out):
    directions = list(DIRECTIONS.items())
    patterns = [(85, 1.5, False), (135, 2.2, True), (180, .7, True)] if held_out else [
        (95, 1.8, False), (165, 1.0, True),
    ]
    for wait in (.3, .5):
        for name, angle in directions:
            for tilt, push, bent in patterns:
                yield name, {
                    "axis_deg": angle, "tilt_deg": tilt, "push_rad_s": push,
                    "bent": bent, "wait_s": wait,
                }


def evaluate(actor, seed_start, held_out, env_kwargs):
    env = GatedRecoveryEnv(**env_kwargs)
    rows = []
    cases = list(fall_cases(held_out))
    try:
        for i, (direction, case) in enumerate(cases):
            seed = seed_start + i % (len(cases) // 2)
            obs, _ = env.reset(seed=seed, options=case)
            while True:
                action, _ = actor.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = env.step(action)
                if terminated or truncated:
                    break
            rows.append({
                "direction": direction, "seed": seed,
                "elapsed_since_release_s": env.data.time - env.release_at,
                "recovery_policy_s": env.steps * .02,
                **info,
            })
    finally:
        env.close()
    by_wait = {}
    for wait in (.3, .5):
        cases = [row for row in rows if row["wait_s"] == wait]
        triggered = [row for row in cases if row["policy_start_s"] is not None]
        recoveries = [row for row in cases if row["is_success"]]
        by_wait[str(wait)] = {
            "episodes": len(cases), "policy_started": len(triggered),
            "fall_detected": sum(row["fall_detected_s"] is not None for row in cases),
            "recovered": len(recoveries),
            "control_faults": sum(row["control_fault"] for row in cases),
            "no_trigger_reasons": dict(Counter(row["no_trigger_reason"] for row in cases
                                               if row["no_trigger_reason"])),
            "max_gate_time_s": max((row["policy_start_s"] for row in triggered), default=None),
            "earliest_start_after_quiet_s": min(
                (row["policy_start_s"] - row["quiet_since_s"] for row in triggered), default=None),
            "commands_before_wait": sum(
                row["first_command_change_s"] is not None and
                row["first_command_change_s"] - row["quiet_since_s"] < wait - 1e-8
                for row in triggered
            ),
            "mean_capped_policy_time_s": float(np.mean([
                row["recovery_policy_s"] if row["is_success"] else 30. for row in cases
            ])),
        }
    return {"summary": by_wait, "trials": rows}


def score(report):
    wait = report["summary"]["0.3"]
    return wait["recovered"], -wait["mean_capped_policy_time_s"]


class Progress(BaseCallback):
    def __init__(self, out):
        super().__init__()
        self.out = out
        self.last = 0
        self.start = time.monotonic()

    def _on_step(self):
        if self.num_timesteps - self.last >= 8192:
            self.last = self.num_timesteps
            progress = {"status": "training", "steps": self.last,
                        "elapsed_s": time.monotonic() - self.start}
            save(self.out / "progress.json", progress)
            print(json.dumps(progress), flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--actor", type=Path, default=INITIAL)
    parser.add_argument("--steps", type=int, default=1048576)
    parser.add_argument("--envs", type=int, default=8)
    parser.add_argument("--checkpoint-every", type=int, default=131072)
    parser.add_argument("--seed", type=int, default=20261111)
    parser.add_argument("--eval-seed", type=int, default=114000)
    parser.add_argument("--still-filter-s", type=float, default=.08)
    parser.add_argument("--physics-timestep", type=float, choices=(.001, .0005), default=.001)
    parser.add_argument("--max-wait", type=float, default=10.)
    args = parser.parse_args()
    if args.envs < 1 or args.steps < 0 or args.checkpoint_every < 1 or args.still_filter_s < 0 or args.max_wait < .5:
        parser.error("require nonnegative steps and filter time, positive envs and checkpoint interval, and max wait of at least 0.5 s")

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    args.out.mkdir(parents=True, exist_ok=True)
    initial = PPO.load(args.actor, device="cpu")
    env_kwargs = {"filter_s": args.still_filter_s, "physics_dt": args.physics_timestep,
                  "max_wait_s": args.max_wait}
    report = {
        "status": "evaluating_initial", "host": platform.node(),
        "requested_steps": args.steps, "envs": args.envs,
        "training_seed": args.seed, "held_out_seed_start": args.eval_seed,
        "initial_actor": str(args.actor), "stages": [],
        "hardware_commands_sent": False, "gate": "candidate with 45 degree detection",
        "still_filter_s": args.still_filter_s,
        "physics_timestep_s": args.physics_timestep,
        "max_wait_s": args.max_wait,
    }
    save(args.out / "training_report.json", report)
    baseline = evaluate(initial, args.eval_seed, held_out=True, env_kwargs=env_kwargs)
    save(args.out / "held_out_initial.json", baseline)
    print(json.dumps({"baseline": baseline["summary"]}), flush=True)
    if not args.steps:
        report.update(status="baseline_complete", held_out_initial=baseline["summary"])
        save(args.out / "training_report.json", report)
        save(args.out / "progress.json", {"status": "baseline_complete", "steps": 0})
        return

    vector = make_vec_env(
        GatedRecoveryEnv, n_envs=args.envs, seed=args.seed,
        env_kwargs=env_kwargs,
        vec_env_cls=SubprocVecEnv, vec_env_kwargs={"start_method": "spawn"},
    )
    vector = VecNormalize(vector, norm_obs=False, norm_reward=True, clip_reward=10, gamma=.995)
    try:
        model = PPO.load(args.actor, env=vector, device="cpu", seed=args.seed,
                         custom_objects={"learning_rate": 6e-5})
        model.num_timesteps = 0
        selected = 0
        best = score(evaluate(initial, 112000, held_out=False, env_kwargs=env_kwargs))
        initial.save(args.out / "best")
        progress = Progress(args.out)
        while model.num_timesteps < args.steps:
            target = min(args.checkpoint_every, args.steps - model.num_timesteps)
            model.learn(total_timesteps=target, reset_num_timesteps=False, callback=progress)
            steps = model.num_timesteps
            dev = evaluate(model, 112000, held_out=False, env_kwargs=env_kwargs)
            trained = score(dev)
            if trained > best:
                best, selected = trained, steps
                model.save(args.out / "best")
            model.save(args.out / f"checkpoint_{steps:08d}")
            vector.save(args.out / "normalization_latest.pkl")
            report["stages"].append({"steps": steps, "development": dev})
            report.update(status="training", selected_steps=selected, actual_steps=steps)
            save(args.out / "training_report.json", report)
            print(json.dumps({"steps": steps, "selected_steps": selected,
                              "development": dev["summary"]}), flush=True)
        selected_actor = PPO.load(args.out / "best.zip", device="cpu")
        held_out = evaluate(selected_actor, args.eval_seed, held_out=True, env_kwargs=env_kwargs)
        save(args.out / "held_out_selected.json", held_out)
        report.update(status="complete", held_out_initial=baseline["summary"],
                      held_out_selected=held_out["summary"])
        save(args.out / "training_report.json", report)
        save(args.out / "progress.json", {"status": "complete", "steps": model.num_timesteps})
        print(json.dumps({"status": "complete", "steps": model.num_timesteps,
                          "selected_steps": selected, "held_out": held_out["summary"]}), flush=True)
    finally:
        vector.close()


if __name__ == "__main__":
    main()
