"""Watch a trained policy in the MuJoCo viewer and drive it with the keyboard.

    uv run python play.py runs/<run>/final.zip
    uv run python play.py runs/<run>/final.zip --no-random   # nominal robot, no pushes/noise
    uv run python play.py runs/<run>/final.zip --step 0.03   # step terrain, 3 cm steps

Keys (click the viewer window first):
    ↑ / ↓   forward / backward        ← / →   turn left / right
    , / .   sidestep left / right      space   stop (stand still)
    + / -   step height ±0.5 cm (with --step; the robot restarts in the middle)
Ctrl + right-drag on the body pushes the robot.

Heading hold (on by default, like the real robot should run it): when the turn command is zero, a P loop on
the IMU yaw adds a small turn command that cancels drift, so "forward" really goes straight.
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np
from stable_baselines3 import PPO

from muto_env.env import CMD_MAX, EnvConfig, MutoEnv

HEADING_GAIN = 2.0  # [rad/s per rad] heading-hold P gain
STEP = CMD_MAX / 3  # each key press changes the command by a third of its range
KEYS = {265: (0, +1), 264: (0, -1), 44: (1, +1), 46: (1, -1), 263: (2, +1), 262: (2, -1)}  # GLFW codes


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policy")
    p.add_argument("--no-random", action="store_true", help="disable randomization, noise and pushes")
    p.add_argument("--no-heading-hold", action="store_true", help="let the policy drift (no yaw correction)")
    p.add_argument("--step", type=float, help="step terrain with this step height [m], e.g. 0.03; keys + / - change it")
    args = p.parse_args()

    cfg = EnvConfig(episode_s=1e9)
    if args.no_random:
        cfg = EnvConfig(episode_s=1e9, latency_s=(0, 0), servo_tau_s=(0, 0), imu_tilt_bias_deg=0, gyro_noise=0,
                        gravity_noise=0, payload_kg=(0.35, 0.35), com_shift_m=0, kp_scale=(1, 1), torque_scale=(1, 1),
                        armature=(0.03, 0.03), frictionloss=(0.05, 0.05), damping_scale=(1, 1),
                        foot_friction=(0.8, 0.8), push_vel=0.0, init_joint_noise=0.0)
    policy = PPO.load(args.policy, device="cpu")
    cfg.history = policy.observation_space.shape[0] // 29  # memory policies see the last N frames
    cfg.terrain = args.step is not None
    env = MutoEnv(cfg, render_mode="rgb_array")
    command = np.zeros(3)
    step = {"h": args.step or 0.0, "changed": False}

    def on_key(key):
        if key == 32:
            command[:] = 0
        elif cfg.terrain and key in (61, 45, 334, 333):  # + / - (main keyboard or keypad)
            step["h"] = float(np.clip(step["h"] + (0.005 if key in (61, 334) else -0.005), 0.0, 0.08))
            step["changed"] = True
            print(f"step height {step['h'] * 100:.1f} cm (robot restarts in the middle)")
            return
        elif key in KEYS:
            i, s = KEYS[key]
            command[i] = np.clip(command[i] + s * STEP[i], -CMD_MAX[i], CMD_MAX[i])
        print(f"command: vx={command[0]:+.2f} m/s  vy={command[1]:+.2f} m/s  yaw={command[2]:+.2f} rad/s")

    def yaw():
        q = env.data.sensor("imu_quat").data
        return np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))

    obs, _ = env.reset(options={"command": command, "step_height": step["h"]})
    target_yaw = yaw()
    with mujoco.viewer.launch_passive(env.model, env.data, key_callback=on_key) as viewer:
        viewer.cam.distance, viewer.cam.elevation = 1.0, -20
        while viewer.is_running():
            t0 = time.time()
            env.command = command.copy()
            if command[2] != 0 or not np.any(command) or args.no_heading_hold:
                target_yaw = yaw()  # turning or standing: the current heading becomes the one to hold
            else:
                err = (target_yaw - yaw() + np.pi) % (2 * np.pi) - np.pi
                env.command[2] = np.clip(HEADING_GAIN * err, -CMD_MAX[2], CMD_MAX[2])
            obs = env.observe_now()  # include the command just typed
            action, _ = policy.predict(obs, deterministic=True)
            obs, _, fell, _, _ = env.step(action)
            if fell or step["changed"]:
                print("fell - resetting" if fell else "new terrain")
                obs, _ = env.reset(options={"command": command, "step_height": step["h"]})
                step["changed"] = False
                target_yaw = yaw()
            viewer.cam.lookat[:] = env.data.qpos[:3]
            viewer.sync()
            time.sleep(max(0.0, env.dt - (time.time() - t0)))


if __name__ == "__main__":
    main()
