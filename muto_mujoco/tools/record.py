"""Record a GIF of a policy following a sequence of commands (nominal robot, no pushes).

    uv run python tools/record.py runs/<run>/final.zip out.gif
"""
import pathlib
import sys

import numpy as np
from PIL import Image, ImageDraw
from stable_baselines3 import PPO

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from muto_env.env import EnvConfig, MutoEnv  # noqa: E402

SCRIPT = [("forward", (0.15, 0, 0), 3.0), ("turn left", (0, 0, 0.6), 2.5), ("sidestep", (0, 0.1, 0), 2.5),
          ("stand", (0, 0, 0), 2.0)]

policy = PPO.load(sys.argv[1], device="cpu")
env = MutoEnv(EnvConfig(push_vel=0.0, episode_s=1e9), render_mode="rgb_array", seed=0)
obs, _ = env.reset(seed=2, options={"command": SCRIPT[0][1]})
frames, joints_stand = [], []
for label, cmd, secs in SCRIPT:
    env.command = np.array(cmd, float)
    for k in range(int(secs * 50)):
        obs = env._obs()
        obs, *_ = env.step(policy.predict(obs, deterministic=True)[0])
        if label == "stand" and k > 25:
            joints_stand.append(env.data.qpos[env.qadr].copy())
        if k % 2 == 0:  # 25 fps
            img = Image.fromarray(env.render())
            ImageDraw.Draw(img).text((10, 10), f"{label}  cmd={cmd}", fill=(255, 255, 255))
            frames.append(img.resize((480, 360)))
frames[0].save(sys.argv[2], save_all=True, append_images=frames[1:], duration=40, loop=0)
print(f"saved {sys.argv[2]} ({len(frames)} frames)")
j = np.array(joints_stand)
print(f"standing: joint motion std {np.degrees(j.std(0)).max():.2f} deg (max over joints), "
      f"mean |q - home| {np.degrees(np.abs(j.mean(0) - env.home_ctrl)).max():.1f} deg")
