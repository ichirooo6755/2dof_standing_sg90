"""Edge-balance feasibility diagnostic using MuJoCo joint and base state.

Linearize about a planar edge pivot for feedback design. All evaluations run
the free robot with the original mesh contacts and servo model. This controller
uses simulated joint encoders and is for simulation only.
"""

import json
from pathlib import Path
import argparse

import mujoco
import numpy as np

from envs import EdgeEnv, HERE


def linearize(env):
    pitch = env.target_pitch
    model = env.model
    data = mujoco.MjData(model)
    model_mass = np.empty((model.nv, model.nv))

    def pose(angle, joint):
        data.qpos[3:7] = [np.cos(angle / 2), 0, np.sin(angle / 2), 0]
        data.qpos[-1] = joint
        mujoco.mj_forward(model, data)
        return data.xmat[env.base_id].reshape(3, 3)

    rotation = pose(pitch, 0.)
    support = env.neutral_support
    heights = support @ rotation[2]
    pivot = support[heights < heights.min() + 0.00015].mean(axis=0)
    jacobian = np.zeros((model.nv, 2))
    jacobian[4, 0] = 1.
    jacobian[-1, 1] = 1.

    def gravity(angle, joint):
        rotation = pose(angle, joint)
        jacobian[:3, 0] = np.cross([0, 1, 0], -rotation @ pivot)
        return jacobian.T @ data.qfrc_bias

    bias = gravity(pitch, 0.)
    stiffness = np.column_stack([
        (gravity(pitch + .0001, 0.) - gravity(pitch - .0001, 0.)) / .0002,
        (gravity(pitch, .0001) - gravity(pitch, -.0001)) / .0002,
    ])
    gravity(pitch, 0.)
    mujoco.mj_fullM(model, model_mass, data.qM)
    inverse_mass = np.linalg.inv(jacobian.T @ model_mass @ jacobian)
    stiffness[1, 1] += env.kp
    continuous = np.zeros((5, 5))
    continuous[:2, 2:4] = np.eye(2)
    continuous[2:4, :2] = -inverse_mass @ stiffness
    continuous[2:4, 2:4] = -inverse_mass @ np.diag([0., .013])
    continuous[2:4, 4] = inverse_mass[:, 1] * env.kp
    # Use ZOH for the 20 ms command cycle with the nominal one-cycle delay.
    term = transition = np.eye(5)
    for n in range(1, 32):
        term = term @ (continuous * .02 / n)
        transition = transition + term
    input_matrix = np.array([[0., 0., 0., 0., .1]]).T
    return transition, input_matrix, bias[1] / env.kp


def gain(transition, control):
    weights = np.diag([50., .5, 1., .025, 1.])
    cost = weights.copy()
    for _ in range(1000):
        feedback = (control.T @ cost @ transition) / (1. + (control.T @ cost @ control).item())
        closed_loop = transition - control @ feedback
        updated = weights + feedback.T @ feedback + closed_loop.T @ cost @ closed_loop
        if np.max(np.abs(updated - cost)) < 1e-9:
            break
        cost = updated
    return feedback[0]


def run(env, params, seed, randomize):
    env.randomize = randomize
    env.reset(seed=seed, options={"start": "near"})
    feedback, pitch_offset, neutral = params[:5], params[5], params[6]
    reference = env.target_pitch + pitch_offset
    total = 0.
    for _ in range(env.horizon):
        rotation = env.data.xmat[env.base_id].reshape(3, 3)
        pitch = np.arctan2(-rotation[2, 0], rotation[2, 2])
        state = np.array([pitch - reference, env.data.qpos[-1], env.data.qvel[4],
                          env.data.qvel[-1], env.command[1] - neutral])
        action = [np.clip(-env.command[0] / .1, -.4, .4), -feedback @ state]
        _, reward, terminated, truncated, info = env.step(action)
        total += reward
        if terminated or truncated:
            break
    return {**info, "seed": seed, "score": total / env.horizon + 8. * info["edge_seconds"],
            "seconds": env.steps * .02}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=HERE / "results/edge/state_feedback")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    env = EdgeEnv(randomize=False)
    env.horizon = 250
    env.reset(seed=96200)
    transition, control, neutral = linearize(env)
    feedback = gain(transition, control)
    original = np.r_[feedback, 0., neutral]
    nominal = [run(env, original, seed, False) for seed in range(96200, 96206)]
    print(json.dumps({"nominal_initial": nominal, "gains": feedback.tolist()}, indent=2), flush=True)

    rng = np.random.default_rng(20261111)
    std = np.r_[np.maximum(.08, np.abs(feedback) * .25), .012, .02]
    mean = best = original
    best_score = -np.inf
    generations = []
    for generation in range(16):
        candidates = rng.normal(mean, std, (24, len(original)))
        candidates[0] = best
        seeds = [96210 + generation, 96230 + generation]
        scores = [
            np.mean([run(env, params, seed, i == 1)["score"] for i, seed in enumerate(seeds)])
            for params in candidates
        ]
        order = np.argsort(scores)[::-1]
        elite = candidates[order[:6]]
        mean = .25 * mean + .75 * elite.mean(axis=0)
        std = np.maximum(std * .15, .25 * std + .75 * elite.std(axis=0))
        candidate = candidates[order[0]]
        dev = [run(env, candidate, seed, True) for seed in range(96250, 96256)]
        score = float(np.mean([row["score"] for row in dev]))
        if score > best_score:
            best, best_score = candidate, score
        generations.append({"generation": generation, "development_score": score, "trials": dev})
        print(json.dumps({"generation": generation, "score": score,
                          "max_development_seconds": max(row["edge_seconds"] for row in dev)}), flush=True)
    held = [run(env, best, seed, True) for seed in range(96300, 96324)]
    result = {
        "state_inputs": "simulated base pitch, gyro Y, upper joint angle and velocity, upper joint command",
        "nominal_initial": nominal,
        "initial_parameters": original.tolist(),
        "selected_parameters": best.tolist(),
        "generations": generations,
        "held_out": held,
        "successes": sum(row["is_success"] for row in held),
        "max_edge_seconds": max(row["edge_seconds"] for row in held),
        "hardware_commands_sent": False,
    }
    (args.out / "diagnostic.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("successes", "max_edge_seconds")}), flush=True)
    env.close()


if __name__ == "__main__":
    main()
