"""Check deploy/policy_io.py against the training environment, then save test vectors for the robot node.

    uv run python deploy/make_test_vectors.py runs/<run>/final.onnx [out.json]   # default deploy/test_vectors.json

1. Runs the policy in MutoEnv (sensor noise off) through stand / walk / turn / sidestep / stand.
   At every step, PolicyIO must rebuild exactly the env's 29 inputs from the raw IMU sample, and give
   exactly the servo degrees the env sent. Any mismatch stops the script.
2. Saves deploy/test_vectors.json: per step the raw inputs (IMU quaternion, gyro, command, clock phase),
   the expected inputs, the ONNX output and the servo degrees. The C++ node replays it in its self-test.
"""
import json
import pathlib
import sys

import numpy as np
import onnxruntime as ort

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from deploy.policy_io import PolicyIO  # noqa: E402
from muto_env.env import EnvConfig, MutoEnv  # noqa: E402

SCRIPT = [((0, 0, 0), 50), ((0.15, 0, 0), 100), ((0, 0, 0.6), 60), ((0, 0.1, 0), 60), ((-0.1, 0, -0.3), 60), ((0, 0, 0), 50)]


def main():
    onnx_path = pathlib.Path(sys.argv[1])
    io = PolicyIO.from_onnx(onnx_path)
    policy = ort.InferenceSession(str(onnx_path))

    env = MutoEnv(EnvConfig(push_vel=0.0, imu_tilt_bias_deg=0.0, gravity_noise=0.0, gyro_noise=0.0), seed=0)
    env.reset(seed=7, options={"command": (0, 0, 0)})
    env.next_command = np.inf  # the script sets the commands
    held = {}  # the IMU sample the robot would hold (the IMU runs at 25 Hz, the policy at 50 Hz)
    read_imu = env._read_imu

    def read_and_keep():
        read_imu()
        held["quat"] = env.data.sensordata[env.quat_adr:env.quat_adr + 4].copy()
        held["gyro"] = env.imu_gyro.copy()
    env._read_imu = read_and_keep
    read_and_keep()
    io.reset(phase=env.gait_phase)

    vectors, worst_obs = [], 0.0
    for command, steps in SCRIPT:
        for _ in range(steps):
            env.command = np.array(command, float)
            phase = io.phase
            obs = io.observation(held["quat"], held["gyro"], command)
            worst_obs = max(worst_obs, np.abs(obs - env._obs()).max())
            assert np.allclose(obs, env._obs(), atol=1e-5), "PolicyIO inputs differ from the env"

            action = policy.run(["action"], {"obs": obs[None]})[0][0]
            servo = io.servo_degrees(action)
            vectors.append(dict(imu_quat_wxyz=held["quat"].tolist(), gyro_rad_s=held["gyro"].tolist(),
                                command=list(command), phase=phase, obs=obs.tolist(),
                                action=action.tolist(), servo_deg=servo.tolist()))
            env.step(action)
            io.advance(action)
            env_servo = np.round(io.sign * np.degrees(env.target) + io.offset).astype(int)
            assert np.array_equal(servo, env_servo), f"servo degrees differ: {servo} vs {env_servo}"

    # heading hold: a few hand-made cases (command in, heading change, command out)
    heading = []
    for cmd, yaw_deg in [((0.15, 0, 0), 0), ((0.15, 0, 0), -10), ((0.15, 0, 0), 30), ((0, 0, 0.6), 5), ((0, 0, 0), 5)]:
        io.target_yaw = 0.0
        q = [np.cos(np.radians(yaw_deg) / 2), 0, 0, np.sin(np.radians(yaw_deg) / 2)]
        heading.append(dict(target_yaw=0.0, command=list(cmd), imu_quat_wxyz=q,
                            command_out=io.hold_heading(cmd, q).tolist()))

    out = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "deploy" / "test_vectors.json"
    out.write_text(json.dumps(dict(onnx=onnx_path.name, interface=io.cfg, steps=vectors, heading_hold=heading)))
    walking = sum(1 for v in vectors if any(v["command"]))
    print(f"PolicyIO matches the env on {len(vectors)} steps (max input difference {worst_obs:.1e}, "
          f"servo degrees identical). Wrote {out} ({walking} walking steps, {len(heading)} heading cases).")


if __name__ == "__main__":
    main()
