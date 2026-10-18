"""Export a trained policy to ONNX for the robot's muto_policy node.

    uv run python export_onnx.py runs/<run>/final.zip            # writes runs/<run>/final.onnx

The ONNX model maps obs (1, 29) -> action (1, 18) in [-1, 1] (deterministic, mean action).
The robot side must then do exactly what MutoEnv.step does (see the interface in muto_env/env.py):
    target_rad = home + ACTION_SCALE * action   (clip to joint range, servo-ID order)
    servo_deg  = round(SERVO_SIGN * deg(target_rad) + SERVO_OFFSET_DEG)
"""

import argparse
import json
import pathlib

import numpy as np
import onnx
import onnxruntime as ort
import torch
from stable_baselines3 import PPO

from muto_env import robot
from muto_env.env import ACTION_SCALE, CMD_MAX, GAIT_HZ, GYRO_SCALE, MutoEnv


class DeterministicPolicy(torch.nn.Module):
    def __init__(self, sb3_policy):
        super().__init__()
        self.extractor = sb3_policy.mlp_extractor.policy_net
        self.action_net = sb3_policy.action_net

    def forward(self, obs):
        return torch.clamp(self.action_net(self.extractor(obs)), -1.0, 1.0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policy")
    args = p.parse_args()
    src = pathlib.Path(args.policy)
    out = src.with_suffix(".onnx")

    model = PPO.load(src, device="cpu")
    net = DeterministicPolicy(model.policy).eval()
    env = MutoEnv()
    dummy = torch.zeros(1, env.observation_space.shape[0])
    torch.onnx.export(net, dummy, out, input_names=["obs"], output_names=["action"], dynamo=False,
                      dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}}, opset_version=17)

    # the interface travels with the model, so the robot side can check it
    interface = dict(
        obs=["gravity_x", "gravity_y", "gravity_z", "gyro_x*0.25", "gyro_y*0.25", "gyro_z*0.25"]
        + [f"prev_action_{j}" for j in robot.JOINT_NAMES] + ["cmd_vx/max", "cmd_vy/max", "cmd_yaw/max"]
        + ["sin(2pi*gait_hz*t)", "cos(2pi*gait_hz*t)"], gait_hz=GAIT_HZ,
        action=robot.JOINT_NAMES, action_scale_rad=ACTION_SCALE, gyro_scale=GYRO_SCALE, cmd_max=CMD_MAX.tolist(),
        home_rad=env.home_ctrl.tolist(), joint_range_rad=[env.ctrl_lo.tolist(), env.ctrl_hi.tolist()],
        servo_sign=list(robot.SERVO_SIGN), servo_offset_deg=list(robot.SERVO_OFFSET_DEG),
        control_hz=robot.CONTROL_HZ, imu_hz=robot.IMU_HZ)
    m = onnx.load(out)
    m.metadata_props.add(key="muto_interface", value=json.dumps(interface))
    onnx.save(m, out)

    # check ONNX == PyTorch on random observations
    obs = np.random.default_rng(0).normal(size=(64, dummy.shape[1])).astype(np.float32)
    onnx_act = ort.InferenceSession(str(out)).run(["action"], {"obs": obs})[0]
    torch_act = model.predict(obs, deterministic=True)[0]
    err = np.abs(onnx_act - np.clip(torch_act, -1, 1)).max()
    print(f"wrote {out} (max |onnx - torch| = {err:.2e})")


if __name__ == "__main__":
    main()
