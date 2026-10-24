"""Render a presentation video (MP4) of a policy following a scripted sequence of commands.

    MUJOCO_GL=osmesa uv run python tools/video.py runs/<run>/final.zip out.mp4
    MUJOCO_GL=osmesa uv run python tools/video.py runs/<run>/final.zip out.mp4 --width 1280 --height 720
    MUJOCO_GL=osmesa uv run python tools/video.py runs/<run>/final.zip out.mp4 --step 0.03   # walk forward over steps

Smooth following camera, small captions, heading hold on, randomized robot without pushes.
Needs ffmpeg on the PATH.
"""

import argparse
import pathlib
import subprocess
import sys
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from stable_baselines3 import PPO

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from muto_env import robot  # noqa: E402
from muto_env.env import CMD_MAX, EnvConfig, MutoEnv  # noqa: E402

# (caption, command (vx, vy, yaw rate), seconds)
SCRIPT = [
    ("Standing", (0.0, 0.0, 0.0), 2.0),
    ("Walking forward", (0.15, 0.0, 0.0), 4.0),
    ("Turning left", (0.0, 0.0, 0.6), 3.0),
    ("Walking sideways", (0.0, 0.1, 0.0), 3.0),
    ("Turning right", (0.0, 0.0, -0.6), 3.0),
    ("Walking backward", (-0.15, 0.0, 0.0), 3.0),
    ("Stop", (0.0, 0.0, 0.0), 2.5),
]
FONT = "/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf"
FONT_SMALL = "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf"
FPS = 25  # policy runs at 50 Hz; every 2nd step is a frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("out")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (for a quick test)")
    ap.add_argument("--step", type=float, help="step terrain with this step height [m]: walk forward for 20 s")
    args = ap.parse_args()

    script = SCRIPT if args.step is None else [(f"Walking forward over {args.step * 100:.0f} cm steps", (0.15, 0.0, 0.0), 20.0)]
    policy = PPO.load(args.policy, device="cpu")
    cfg = EnvConfig(push_vel=0.0, episode_s=1e9, terrain=args.step is not None,
                    history=policy.observation_space.shape[0] // 29)
    env = MutoEnv(cfg, render_mode="rgb_array", seed=args.seed)
    m = env.model
    m.vis.global_.offwidth, m.vis.global_.offheight = args.width, args.height
    m.vis.quality.shadowsize = 4096
    m.vis.quality.offsamples = 4
    renderer = mujoco.Renderer(m, args.height, args.width)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.85, -18, 150
    opt = mujoco.MjvOption()

    obs, _ = env.reset(seed=args.seed, options={"command": (0, 0, 0), "step_height": args.step or 0.0})
    lookat = env.data.qpos[:3].copy()

    big = ImageFont.truetype(FONT, args.height // 22)
    small = ImageFont.truetype(FONT_SMALL, args.height // 40)
    ffmpeg = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{args.width}x{args.height}", "-r", str(FPS), "-i", "-",
         "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
         args.out], stdin=subprocess.PIPE)

    def yaw():
        q = env.data.sensor("imu_quat").data
        return np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))

    n_frames, t_start, step = 0, time.time(), 0
    total_steps = int(sum(s for *_, s in script) * robot.CONTROL_HZ)
    for caption, cmd, secs in script:
        target_yaw = yaw()
        for k in range(int(secs * robot.CONTROL_HZ)):
            env.command = np.array(cmd, float)
            if cmd[2] == 0 and any(cmd):  # heading hold, as on the real robot
                err = (target_yaw - yaw() + np.pi) % (2 * np.pi) - np.pi
                env.command[2] = np.clip(2.0 * err, -CMD_MAX[2], CMD_MAX[2])
            else:
                target_yaw = yaw()
            obs, *_ = env.step(policy.predict(env.observe_now(), deterministic=True)[0])
            step += 1
            if step % (robot.CONTROL_HZ // FPS):
                continue

            # camera: follows smoothly, slowly circles the robot
            lookat += 0.08 * (env.data.qpos[:3] - lookat)
            cam.lookat[:] = lookat + [0, 0, 0.02]
            cam.azimuth = 150 + 40 * np.sin(2 * np.pi * step / total_steps)
            renderer.update_scene(env.data, cam, scene_option=opt)
            img = Image.fromarray(renderer.render())

            d = ImageDraw.Draw(img, "RGBA")
            pad = args.height // 30
            fade = min(1.0, k / 12) if k < 12 else 1.0
            d.text((pad, args.height - pad - big.size - small.size - 10), caption, font=big,
                   fill=(255, 255, 255, int(255 * fade)))
            speed = f"{cmd[0]:+.2f} m/s forward" if cmd[0] else (f"{cmd[1]:+.2f} m/s sideways" if cmd[1] else
                                                                 (f"{cmd[2]:+.1f} rad/s turn" if cmd[2] else "zero command"))
            d.text((pad, args.height - pad - small.size), f"command: {speed}", font=small, fill=(220, 220, 220, 230))
            d.text((args.width - pad, pad), "MuJoCo simulation · learned policy (PPO)", font=small,
                   fill=(230, 230, 230, 220), anchor="ra")
            ffmpeg.stdin.write(np.asarray(img).tobytes())
            n_frames += 1
            if n_frames % 50 == 0:
                print(f"{n_frames} frames, {(time.time() - t_start) / n_frames:.2f} s/frame", flush=True)
            if args.frames and n_frames >= args.frames:
                break
        if args.frames and n_frames >= args.frames:
            break
    ffmpeg.stdin.close()
    ffmpeg.wait()
    print(f"saved {args.out}: {n_frames} frames ({n_frames / FPS:.1f} s)")


if __name__ == "__main__":
    main()
