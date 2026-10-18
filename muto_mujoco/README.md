# MUTO hexapod — MuJoCo

MuJoCo version of the MUTO RL project (the Isaac Lab version lives in `../MUTO_RL/`).

## Setup

```bash
cd muto_mujoco
uv sync                                   # creates .venv with mujoco and the RL libraries
uv run python tools/prepare_meshes.py     # copies the robot meshes into assets/meshes/ (not in git)
uv run python tools/build_mjcf.py         # regenerates model/muto.xml
```

## Look at the robot

```bash
uv run python -m mujoco.viewer --mjcf model/scene.xml
```

In the viewer: double-click a body to select it, Ctrl+right-drag to push it,
the *Control* panel on the right has a slider per servo. Press `3` to show
collision shapes, `2` to hide meshes.

## Train, watch, export

```bash
uv run python train.py                              # PPO on 6 CPU processes, ~50 min for 5M steps
uv run tensorboard --logdir runs                    # progress at http://localhost:6006
uv run python play.py runs/<run>/final.zip          # viewer: arrow keys / , . to steer, space to stop
uv run python export_onnx.py runs/<run>/final.zip   # -> final.onnx for the robot's muto_policy node
```

The task: walk on velocity commands (vx, vy, yaw rate); a zero command means stand still in the
home pose. The policy interface (what the robot must reproduce) is documented at the top of
`muto_env/env.py` and embedded in the exported ONNX as metadata (`muto_interface`).

## Files

| Path | What |
|---|---|
| `model/muto.xml` | robot + `home` keyframe (Yahboom standby pose); generated, don't edit by hand |
| `model/scene.xml` | floor and light; includes `muto.xml` |
| `assets/meshes/` | full-detail STL meshes; ones over MuJoCo's 200k-face limit are split into `_0`, `_1`, … chunks |
| `tools/prepare_meshes.py` | rebuilds `assets/meshes/` from `../assets/yahboomcar_description/meshes/` |
| `tools/build_mjcf.py` | rebuilds `model/muto.xml` from `muto_des.urdf`; servo model + masses live here (each value marked DATASHEET or ESTIMATE) |
| `tools/render.py` | saves a PNG (visual + collision view) |
| `muto_env/robot.py` | real-robot facts: servo-ID order, servo↔URDF angle conversion, rates |
| `muto_env/env.py` | Gymnasium environment: observation/action interface, rewards, randomization |
| `train.py` / `play.py` / `export_onnx.py` | train with PPO / watch and steer / export to ONNX |

After a new SolidWorks export: `uv run python tools/prepare_meshes.py && uv run python tools/build_mjcf.py`.

## Robot facts used

- Leg names are pinyin: **z** = 左 left (+y), **y** = 右 right (−y); **q** front, **z** middle, **h** rear.
  Joint 1 = hip yaw, 2 = hip pitch, 3 = knee.
- Actuators are ordered by real servo ID: 1–3 `yq`, 4–6 `yz`, 7–9 `yh`, 10–12 `zh`, 13–15 `zz`, 16–18 `zq`.
- Servo: Yahboom 35 kg bus servo, 7.4 V: stall 3.43 N·m, no-load 0.22 s/60°, 61.5 g, software range ±90°.
- Still unknown, needs the real robot: total mass (weigh it and set `MEASURED_TOTAL_MASS`),
  servo P-gain and armature, and the sign convention between Yahboom angles and URDF angles.
