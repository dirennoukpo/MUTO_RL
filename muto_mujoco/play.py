"""Watch a trained policy in the MuJoCo viewer and drive it with the keyboard.

    uv run python play.py runs/<run>/final.zip
    uv run python play.py runs/<run>/final.zip --no-random   # nominal robot, no pushes/noise

Keys (click the viewer window first):
    ↑ / ↓   forward / backward        ← / →   turn left / right
    , / .   sidestep left / right      space   stop (stand still)
Ctrl + right-drag on the body pushes the robot.
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np
from stable_baselines3 import PPO

from muto_env.env import CMD_MAX, EnvConfig, MutoEnv

STEP = CMD_MAX / 3  # each key press changes the command by a third of its range
KEYS = {265: (0, +1), 264: (0, -1), 44: (1, +1), 46: (1, -1), 263: (2, +1), 262: (2, -1)}  # GLFW codes


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policy")
    p.add_argument("--no-random", action="store_true", help="disable randomization, noise and pushes")
    args = p.parse_args()

    cfg = EnvConfig(episode_s=1e9)
    if args.no_random:
        cfg = EnvConfig(episode_s=1e9, latency_s=(0, 0), servo_tau_s=(0, 0), imu_tilt_bias_deg=0, gyro_noise=0,
                        gravity_noise=0, payload_kg=(0.35, 0.35), com_shift_m=0, kp_scale=(1, 1), torque_scale=(1, 1),
                        armature=(0.03, 0.03), frictionloss=(0.05, 0.05), damping_scale=(1, 1),
                        foot_friction=(0.8, 0.8), push_vel=0.0, init_joint_noise=0.0)
    env = MutoEnv(cfg, render_mode="rgb_array")
    policy = PPO.load(args.policy, device="cpu")
    command = np.zeros(3)

    def on_key(key):
        if key == 32:
            command[:] = 0
        elif key in KEYS:
            i, s = KEYS[key]
            command[i] = np.clip(command[i] + s * STEP[i], -CMD_MAX[i], CMD_MAX[i])
        print(f"command: vx={command[0]:+.2f} m/s  vy={command[1]:+.2f} m/s  yaw={command[2]:+.2f} rad/s")

    obs, _ = env.reset(options={"command": command})
    with mujoco.viewer.launch_passive(env.model, env.data, key_callback=on_key) as viewer:
        viewer.cam.distance, viewer.cam.elevation = 1.0, -20
        while viewer.is_running():
            t0 = time.time()
            env.command = command.copy()
            obs = env._obs()
            action, _ = policy.predict(obs, deterministic=True)
            obs, _, fell, _, _ = env.step(action)
            if fell:
                print("fell - resetting")
                obs, _ = env.reset(options={"command": command})
            viewer.cam.lookat[:] = env.data.qpos[:3]
            viewer.sync()
            time.sleep(max(0.0, env.dt - (time.time() - t0)))


if __name__ == "__main__":
    main()
