"""Step-climbing test: for each step height, can the policy climb two steps up?

    uv run python tools/steps_eval.py runs/<run>/final.zip [more runs ...] [--robots 10]

Each robot starts in the flat middle with a random heading and is told to walk forward at 0.15 m/s for 15 s
(no pushes). Success = the body gets past the second ring (two steps up, 0.70 m from the middle) without
falling. Also reports flat-ground speed tracking (same commands as tools/ablate_eval.py).
Memory policies are detected from the observation size.
"""
import argparse
import pathlib
import sys
from multiprocessing import Pool

import numpy as np
from stable_baselines3 import PPO

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from muto_env import terrain  # noqa: E402
from muto_env.env import EnvConfig, MutoEnv  # noqa: E402

HEIGHTS_CM = [0, 1, 2, 3, 4, 5]
GOAL = terrain.CENTER + 2 * terrain.TREAD
FLAT = {"forward": (0.15, 0, 0), "slow fwd": (0.07, 0, 0), "backward": (-0.15, 0, 0), "left": (0, 0.1, 0),
        "turn left": (0, 0, 0.6), "turn right": (0, 0, -0.6)}


def test(job):
    path, n = job
    policy = PPO.load(path, device="cpu")
    hist = policy.observation_space.shape[0] // 29
    env = MutoEnv(EnvConfig(terrain=True, history=hist, push_vel=0.0), seed=0)
    out = {}
    for h in HEIGHTS_CM:
        ok = falls = 0
        for r in range(n):
            obs, _ = env.reset(seed=100 + r, options={"command": (0.15, 0, 0), "step_height": h / 100})
            for _ in range(750):
                obs, _, fell, _, _ = env.step(policy.predict(obs, deterministic=True)[0])
                if fell or env.episode["reach"] > GOAL:
                    break
            falls += fell
            ok += (not fell) and env.episode["reach"] > GOAL
        out[h] = (ok / n, falls / n)
    err = []
    for name, cmd in FLAT.items():
        for r in range(n // 2):
            obs, _ = env.reset(seed=200 + r, options={"command": cmd, "step_height": 0.0})
            v = []
            for k in range(400):
                obs, *_ = env.step(policy.predict(obs, deterministic=True)[0])
                d = env.data
                if k >= 100:
                    v.append([*(env._rot(d.qpos[3:7]).T @ d.qvel[:3])[:2], d.qvel[5]])
            v = np.mean(v, axis=0)
            err.append(np.hypot(*(v[:2] - cmd[:2])) if cmd[2] == 0 else abs(v[2] - cmd[2]) / 4)
    return str(path), hist, out, float(np.mean(err))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policies", nargs="+")
    ap.add_argument("--robots", type=int, default=10)
    args = ap.parse_args()
    with Pool(len(args.policies)) as pool:
        results = pool.map(test, [(p, args.robots) for p in args.policies])
    print(f"\nclimb 2 steps up, {args.robots} robots per height: success % (falls %)")
    print(f"{'policy':34s} {'mem':>3s} " + " ".join(f"{h:>4d} cm  " for h in HEIGHTS_CM) + "  flat error")
    for path, hist, out, flat in results:
        name = pathlib.Path(path).parent.name.split("_", 2)[-1]
        print(f"{name:34s} {hist:3d} " + " ".join(f"{100 * s:4.0f} ({100 * f:2.0f})" for s, f in out.values())
              + f"   {flat:.3f}")
    print("flat error = mean speed error over 6 commands [m/s] (turn errors divided by 4 to be comparable)")


if __name__ == "__main__":
    main()
