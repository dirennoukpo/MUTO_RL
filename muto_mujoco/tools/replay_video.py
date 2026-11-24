"""Replay the commands a real-robot clip received, in simulation, filmed like the phone (for a sim/real comparison).

    MUJOCO_GL=osmesa uv run python tools/replay_video.py runs/<run>/final.zip clip.json sim.mp4

clip.json: {"duration": seconds, "segments": [[t_start, [vx, vy, yaw_rate]], ...]} with times in seconds from the start
of the phone video (made from the robot's /cmd_vel bag). Output: 1080x960, 30 fps like the phone, low side view from
the robot's right, camera following. Heading hold on for walking commands, as in the robot node.
"""

import argparse
import json
import pathlib
import subprocess
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from stable_baselines3 import PPO

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from muto_env import robot  # noqa: E402
from muto_env.env import CMD_MAX, EnvConfig, MutoEnv  # noqa: E402

FONT = "/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf"
FPS = 30


def caption(cmd):
    vx, vy, wz = cmd
    if vx: return "Walking forward" if vx > 0 else "Walking backward"
    if vy: return "Walking sideways"
    if wz: return "Turning left" if wz > 0 else "Turning right"
    return "Standing"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("clip")
    ap.add_argument("out")
    ap.add_argument("--width", type=int, default=1080)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--seed", type=int, default=3)
    args = ap.parse_args()

    clip = json.loads(pathlib.Path(args.clip).read_text())
    segs = clip["segments"]
    policy = PPO.load(args.policy, device="cpu")
    env = MutoEnv(EnvConfig(push_vel=0.0, episode_s=1e9), render_mode="rgb_array", seed=args.seed)
    m = env.model
    m.vis.global_.offwidth, m.vis.global_.offheight = args.width, args.height
    m.vis.quality.shadowsize = 4096
    m.vis.quality.offsamples = 4
    renderer = mujoco.Renderer(m, args.height, args.width)
    env.reset(seed=args.seed, options={"command": (0, 0, 0)})

    def yaw():
        q = env.data.sensor("imu_quat").data
        return np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))

    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation = 0.9, -14
    cam.azimuth = np.degrees(yaw()) + 90  # looking along the robot's +y = from its right side, forward = image right
    lookat = env.data.qpos[:3].copy()
    font = ImageFont.truetype(FONT, args.height // 20)
    ffmpeg = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{args.width}x{args.height}", "-r", str(FPS), "-i", "-",
         "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", args.out], stdin=subprocess.PIPE)

    n_steps = int(round(clip["duration"] * robot.CONTROL_HZ))
    frame, target_yaw, seg = 0, yaw(), 0
    for k in range(n_steps):
        t = k / robot.CONTROL_HZ
        while seg + 1 < len(segs) and t >= segs[seg + 1][0]:
            seg += 1
        cmd = segs[seg][1]
        env.command = np.array(cmd, float)
        if cmd[2] == 0 and any(cmd):  # heading hold, as on the real robot
            err = (target_yaw - yaw() + np.pi) % (2 * np.pi) - np.pi
            env.command[2] = np.clip(2.0 * err, -CMD_MAX[2], CMD_MAX[2])
        else:
            target_yaw = yaw()
        env.step(policy.predict(env.observe_now(), deterministic=True)[0])

        lookat += 0.05 * (env.data.qpos[:3] - lookat)
        while frame / FPS <= t + 1e-9:  # 50 Hz physics -> 30 fps video
            cam.lookat[:] = lookat + [0, 0, 0.0]
            renderer.update_scene(env.data, cam)
            img = Image.fromarray(renderer.render())
            d = ImageDraw.Draw(img, "RGBA")
            pad = args.height // 25
            d.text((pad, pad), "Simulation (MuJoCo)", font=font, fill=(255, 255, 255, 235))
            d.text((pad, args.height - pad - font.size), caption(cmd), font=font, fill=(255, 255, 255, 235))
            ffmpeg.stdin.write(np.asarray(img).tobytes())
            frame += 1
    ffmpeg.stdin.close()
    ffmpeg.wait()
    print(f"saved {args.out}: {frame} frames ({frame / FPS:.2f} s)")


if __name__ == "__main__":
    main()
