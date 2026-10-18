"""Measure how well a policy follows fixed velocity commands (randomized robot, no pushes).

heading drift = how far the robot turned beyond what the yaw command asked for, over the 8 s test.

    uv run python tools/evaluate.py runs/<run>/final.zip
"""
import pathlib
import sys

import numpy as np
from stable_baselines3 import PPO

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from muto_env.env import EnvConfig, MutoEnv  # noqa: E402

COMMANDS = {"stand": (0, 0, 0), "forward": (0.15, 0, 0), "slow fwd": (0.07, 0, 0), "backward": (-0.15, 0, 0),
            "left": (0, 0.1, 0), "turn left": (0, 0, 0.6), "turn right": (0, 0, -0.6)}

policy = PPO.load(sys.argv[1], device="cpu")
env = MutoEnv(EnvConfig(push_vel=0.0), seed=0)
print(f"{'command':11s} {'vx':>14s} {'vy':>14s} {'yaw rate':>14s}   steps/s/leg  foot-slip  falls  heading drift")
for name, cmd in COMMANDS.items():
    obs, _ = env.reset(seed=1, options={"command": cmd})
    v, contacts_changes, slip, falls = [], 0, [], 0
    prev_contact = None
    heading = lambda q: np.degrees(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
    turned, prev_h = 0.0, heading(env.data.qpos[3:7])
    for k in range(400):  # 8 s; measure the last 6 s
        obs, r, term, trunc, info = env.step(policy.predict(obs, deterministic=True)[0])
        falls += term
        d = env.data
        R = env._rot(d.qpos[3:7])
        contact = d.sensordata[env.touch_adr] > 1.0
        if k >= 100:
            v.append([*(R.T @ d.qvel[:3])[:2], d.qvel[5]])
            if prev_contact is not None:
                contacts_changes += np.sum(contact & ~prev_contact)
            slip.append(info["foot_slip"])
        prev_contact = contact
        h = heading(d.qpos[3:7])
        turned += (h - prev_h + 180) % 360 - 180
        prev_h = h
    v = np.mean(v, axis=0)
    print(f"{name:11s} " + " ".join(f"{c:+.2f} → {a:+.3f}" for c, a in zip(cmd, v))
          + f"   {contacts_changes / 6 / 6:9.2f}  {np.mean(slip):9.4f}  {falls:5d}  "
          + (f"{turned - np.degrees(cmd[2]) * 8:+6.1f} deg / 8 s" if True else ""))
