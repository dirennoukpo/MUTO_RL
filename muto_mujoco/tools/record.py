"""Record a GIF of a policy following a sequence of commands (randomized robot, no pushes, heading hold on).

    uv run python tools/record.py runs/<run>/final.zip out.gif
    uv run python tools/record.py runs/<run>/final.zip docs/walk.gif --small   # light GIF for the README
"""
import argparse
import pathlib
import sys

import numpy as np
from PIL import Image, ImageDraw
from stable_baselines3 import PPO

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from muto_env.env import CMD_MAX, EnvConfig, MutoEnv  # noqa: E402

SCRIPT = [("forward", (0.15, 0, 0), 3.0), ("turn left", (0, 0, 0.6), 2.5), ("sidestep", (0, 0.1, 0), 2.5),
          ("backward", (-0.15, 0, 0), 2.0), ("stand", (0, 0, 0), 2.0)]

ap = argparse.ArgumentParser()
ap.add_argument("policy")
ap.add_argument("out")
ap.add_argument("--small", action="store_true", help="320x240 at 12.5 fps, a few MB")
args = ap.parse_args()
size, every = ((320, 240), 4) if args.small else ((480, 360), 2)

policy = PPO.load(args.policy, device="cpu")
env = MutoEnv(EnvConfig(push_vel=0.0, episode_s=1e9), render_mode="rgb_array", seed=0)
obs, _ = env.reset(seed=2, options={"command": SCRIPT[0][1]})
frames, joints_stand = [], []


def yaw():
    q = env.data.sensor("imu_quat").data
    return np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))


for label, cmd, secs in SCRIPT:
    target_yaw = yaw()
    for k in range(int(secs * 50)):
        env.command = np.array(cmd, float)
        if cmd[2] == 0 and any(cmd):  # heading hold, as in play.py
            err = (target_yaw - yaw() + np.pi) % (2 * np.pi) - np.pi
            env.command[2] = np.clip(2.0 * err, -CMD_MAX[2], CMD_MAX[2])
        else:
            target_yaw = yaw()
        obs = env._obs()
        obs, *_ = env.step(policy.predict(obs, deterministic=True)[0])
        if label == "stand" and k > 25:
            joints_stand.append(env.data.qpos[env.qadr].copy())
        if k % every == 0:
            img = Image.fromarray(env.render())
            ImageDraw.Draw(img).text((10, 10), f"{label}  cmd={cmd}", fill=(255, 255, 255))
            frames.append(img.resize(size).quantize(colors=48, dither=Image.Dither.NONE))
pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
frames[0].save(args.out, save_all=True, append_images=frames[1:], duration=20 * every, loop=0, optimize=True)
print(f"saved {args.out} ({len(frames)} frames, {pathlib.Path(args.out).stat().st_size / 1e6:.1f} MB)")
j = np.array(joints_stand)
print(f"standing: joint motion std {np.degrees(j.std(0)).max():.2f} deg (max over joints), "
      f"mean |q - home| {np.degrees(np.abs(j.mean(0) - env.home_ctrl)).max():.1f} deg")
