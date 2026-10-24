# Deploying the MuJoCo walking policy on the MUTO RS

For the `feat/ros2_control` branch of MUTO-RS-CHOREGRAGPHY. Three changes, then a test procedure.

The policy interface changed from the Isaac version, so the old `muto_policy` node can't run it:

| | old node (Isaac) | new policy (MuJoCo) |
|---|---|---|
| inputs | 24: joint positions, gyro, accelerometer | 29: gravity direction (from the IMU orientation), gyro × 0.25, previous action, velocity command, gait clock |
| outputs | absolute angles, `0.25 × output` | `home pose + 0.3 × output` (URDF radians) |
| commands | none | `/cmd_vel` (forward, sideways, turn) |
| joint readings | used | **not used** (the bus can't read 18 servos at 50 Hz) |

All constants (home pose, scales, clock frequency, joint order) are stored inside the `.onnx` file and read at startup, so the node can't disagree with the model.

## 1. Replace the policy package

Replace `muto_ws/src/muto_policy/` with the `muto_policy/` folder next to this README. It needs one extra library:

```bash
sudo apt install nlohmann-json3-dev
colcon build --packages-select muto_policy      # ONNX Runtime in /usr/local as before
```

## 2. Apply the hardware patch

```bash
cd MUTO-RS-CHOREGRAGPHY
git apply path/to/bringup.patch
colcon build --packages-select muto_bringup muto_description
```

It changes three things:
- **`servo_speed` 300 → 0.** The field is a move time in ms, not a speed. Yahboom's servo manual says 0 = "move to the specified position with fastest speed". With 300, every 20 ms command is spread over 300 ms, so the servos lag far behind the policy. The plugin no longer forces 0 back to 300; it now warns if the value is above 20 ms.
- **Servo IDs:** left front (`zq`) = 16 to 18 and left rear (`zh`) = 10 to 12, as in Yahboom's docs and MutoLib. The old xacro had them swapped. **Check on the robot (step 4).**
- **Angle conversion in config:** the URDF angles the policy uses aren't servo angles. For femur and tibia, `servo = −deg(q) + offset` (offsets 10.75° and 14.82°, from MutoLib and the CAD). This is now `servo_invert_ids` and `servo_offsets` in the xacro, so `/joint_states` and RViz keep showing real URDF angles.

## 3. Copy the model and run the self-test

```bash
cp muto_walk_policy.onnx install/muto_description/share/muto_description/config/   # ../muto_walk_policy.onnx (v4)
ros2 run muto_policy muto_policy_selftest muto_walk_policy.onnx test_vectors.json   # both in muto_mujoco/deploy/
```

The self-test replays 380 steps recorded in the simulator and checks the 29 inputs, the network output and every servo degree. It must print `PASS`. This runs on the robot's CPU and library versions, so it catches build problems before the robot moves.

### Second policy: step climbing

`muto_steps_policy.onnx` (run `steps_nomem_s1`) has the same 29-input interface, so it runs on the same node. It climbs
3 cm steps in simulation (100% of tests, 50% at 4 cm) but walks slightly less precisely on flat ground than v4.
Self-test it the same way with `test_vectors_steps.json`, then start the node with
`policy_path:=muto_steps_policy.onnx`. Try it only after v4 has passed steps a to e below.

## 4. First run on the robot

Robot **held in the air** (feet free) for steps a to c.

a. **Start:** `ros2 launch muto_bringup bringup.launch.py`. The node waits for `/muto/imu` and `/joint_states`, prints the gravity vector (must be about `(0, 0, -1)` with the robot flat), then moves to the home pose over 2 s.

b. **IMU axes.** Watch the first 3 numbers: `ros2 topic echo /muto_policy/obs`. Body frame: x forward, y left, z up.
   - nose down → the 1st number becomes **positive**
   - left side down → the 2nd number becomes **positive**
   - turn the robot left (counter-clockwise from above) → the 6th number (gyro z) becomes **positive**

   If a sign or an axis is wrong, stop: the IMU-to-body conversion must be fixed first. The policy would fall otherwise.

c. **Servo IDs and directions.** Stop the policy node, then send small offsets from the home pose with `ros2 topic pub --once /leg_controller/commands ...` (order = `controllers.yaml`). Check that `zq` moves the left **front** leg, and that a positive femur angle (`*2_Joint`) lifts the foot **up** (the home pose is +0.71 rad = femur 30° above horizontal).

d. **On the floor, command zero:** it should stand still in the home pose.

e. **Walk:** `ros2 run teleop_twist_keyboard teleop_twist_keyboard`. Set the speeds low first (`z` / `x`): the policy is trained for at most 0.15 m/s forward, 0.10 m/s sideways and 0.6 rad/s turn, and larger commands are clipped.

**Safety:** the node stops the policy and holds the home pose if the body tilts more than 50° or the IMU is silent for 0.2 s. Without a `/cmd_vel` message for 0.5 s, the command goes to zero (stand still). Restart the node to walk again.

## 5. One measurement for the simulator

Film one servo jumping 30° (phone slow-motion is enough) with `servo_speed = 0`. The time it takes is the last unknown in the simulator's servo model.

## Node parameters

| parameter | default | |
|---|---|---|
| `policy_path` | `muto_walk_policy.onnx` | relative = in `muto_description/config/` |
| `heading_hold` | true | keeps the heading when walking straight (without it, ~20° drift per 10 s) |
| `ramp_s` | 2.0 | time to move to the home pose at start |
| `max_tilt_deg` | 50 | fault above this tilt |
| `imu_timeout_s` / `cmd_timeout_s` | 0.2 / 0.5 | |
| `controller_joints` | `controllers.yaml` order | order of `/leg_controller/commands` |
