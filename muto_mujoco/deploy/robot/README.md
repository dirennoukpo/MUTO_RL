# Running the policy on the real MUTO RS

The commands used on 2026-11-21 (first real-robot session). Robot: Raspberry Pi 5, Debian, Docker, user `muto`.
Everything ROS runs in one container (`muto_run`), so no DDS setup is needed.

**Who wrote what.** From Diren's [MUTO-RS-CHOREGRAGPHY](https://github.com/dirennoukpo/MUTO-RS-CHOREGRAGPHY)
(branch `feat/muto_policy`): `muto_bringup` (serial driver `muto_link`, ros2_control hardware plugin, IMU, launch file)
and `muto_description` (URDF, controllers). Ours: `../ros2/bringup.patch` (servo speed, IDs, angle conversion),
the policy node `../ros2/muto_policy`, this Dockerfile (a slim version of Diren's) and everything in `tools/`.

## 1. Build (no motion)

On the laptop, from `muto_mujoco/`:
```bash
deploy/robot/assemble.sh /tmp/muto_rl                  # Diren's packages + our patch + our node + the policies
scp -r /tmp/muto_rl muto@<robot-ip>:muto_rl
```
On the robot (`ssh muto@<robot-ip>`):
```bash
cd ~/muto_rl && docker build -t muto_rl:jazzy .        # ~10 min on the Pi 5
```
Self-test (replays 380 simulator steps through the C++ node code, must print PASS):
```bash
docker run --rm muto_rl:jazzy bash -c "source /muto_ws/install/setup.bash && \
  C=/muto_ws/install/muto_description/share/muto_description/config && \
  ros2 run muto_policy muto_policy_selftest \$C/muto_walk_policy.onnx \$C/test_vectors.json"
```

The serial board: use its stable name, it can change from ttyUSB0 to ttyUSB1 after a reconnect.
```bash
SERIAL=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0:/dev/ttyUSB0
```

## 2. Passive checks (torque OFF, legs limp, nothing moves by itself)

```bash
docker rm -f muto_passive; docker run -d --name muto_passive --network host --device $SERIAL \
  -v ~/muto_rl/tools:/tools muto_rl:jazzy bash -c "source /muto_ws/install/setup.bash && \
  xacro /muto_ws/install/muto_description/share/muto_description/urdf/Muto_complet.urdf.xacro \
  | sed 's|<param name=\"torque_enabled\">true</param>|<param name=\"torque_enabled\">false</param>|' \
  > /tmp/muto_passive.urdf && ros2 launch /tools/passive.launch.py"
docker exec -it muto_passive bash -c "source /opt/ros/jazzy/setup.bash && python3 /tools/watch.py 30"
```
- Tilt the robot nose down / left side down: the gravity printed must change sign as expected.
- Move one foot up by hand: the joint printed must be the right leg (`zq` = left front), femur angle positive.
- `python3 /tools/gyro.py 20` prints the raw IMU gyro peaks.

Stop: `docker rm -f muto_passive`.

## 3. Run the policy (servos powered: robot held in the air first, then on the floor)

```bash
mkdir -p ~/muto_rl/bags
docker rm -f muto_run; docker run -d --name muto_run --network host --device $SERIAL \
  -v ~/muto_rl/tools:/tools -v ~/muto_rl/bags:/bags muto_rl:jazzy \
  bash -c "source /muto_ws/install/setup.bash && ros2 launch muto_bringup bringup.launch.py"
docker logs muto_run 2>&1 | grep -E "start: gravity|home pose reached|FAULT"    # gravity ~ (0, 0, -1)
```
The node ramps to the home pose in 2 s, then the policy runs with command zero (stands).

Record everything (recommended):
```bash
docker exec -d muto_run bash -c "source /opt/ros/jazzy/setup.bash && cd /bags && \
  ros2 bag record -o run1 /joint_states /muto/imu /leg_controller/commands /muto_policy/obs /cmd_vel"
```
Check the policy inputs live: `docker exec -it muto_run bash -c "source /opt/ros/jazzy/setup.bash && python3 /tools/obs.py 30"`

Drive it:
```bash
docker exec -it muto_run bash -c "source /muto_ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```
or play the video sequence (forward, 90 deg left, sideways, 90 deg right, backward): `~/muto_rl/video.sh`,
stop it with `~/muto_rl/stop.sh` (the robot then stands).

## 4. Stop

```bash
docker exec muto_run pkill -INT -f "ros2 bag record"; sleep 3
docker stop -t 15 muto_run && docker rm muto_run
```
Then start passive mode (section 2) for a few seconds to be sure the torque is off.
Copy the data to the laptop: `scp -r muto@<robot-ip>:muto_rl/bags .`

## 5. Measurements without ROS (driver stopped)

```bash
docker run --rm --device $SERIAL -v ~/muto_rl/tools:/tools muto_rl:jazzy bash -c "cd /tools && \
  g++ -O2 -std=c++17 muto_tool.cpp -I/muto_ws/src/muto_bringup/include -L/muto_ws/install/muto_bringup/lib \
  -lmuto_link -Wl,-rpath,/muto_ws/install/muto_bringup/lib -o muto_tool && ./muto_tool battery && ./muto_tool timing"
```
`./muto_tool step 2 -30 30 0.6` moves ONE servo (id 2, a femur) from -30 to +30 deg and logs its angle (CSV).

## Known issues (2026-11-21)

- IMU board mounted turned 180 deg: handled by the node (`imu_yaw_offset_deg`, default 180).
- The gyro in `/muto/imu` only changes about once per second (orientation ~12 Hz). The policy walked anyway; cause not found yet.
- Reading all 18 servos takes 24 ms, which is why the policy does not use joint positions.
- After a reboot the Pi falls back to its hotspot (10.42.0.1); Wi-Fi has to be re-enabled by hand.
