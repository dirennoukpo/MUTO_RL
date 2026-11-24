#!/bin/bash
# Builds the folder that is copied to the robot (run on the laptop, from anywhere):
#   deploy/robot/assemble.sh OUT_DIR
# OUT_DIR/ gets: Dockerfile, tools/, video.sh, stop.sh, and src/ with
#   - muto_bringup, muto_description: Diren's MUTO-RS-CHOREGRAGPHY (branch feat/muto_policy) + our bringup.patch
#   - muto_policy: our policy node (deploy/ros2/muto_policy)
#   - both ONNX policies and their test vectors in muto_description/config/
set -e
OUT=${1:?usage: assemble.sh OUT_DIR}
HERE=$(cd "$(dirname "$0")" && pwd)
D=$HERE/..                      # muto_mujoco/deploy
TMP=$(mktemp -d)

git clone -q --depth 1 --filter=blob:none --sparse --branch feat/muto_policy \
  https://github.com/dirennoukpo/MUTO-RS-CHOREGRAGPHY.git "$TMP/choreo"
cd "$TMP/choreo"
git sparse-checkout set --no-cone '/muto_ws/src/muto_bringup/' '/muto_ws/src/muto_description/urdf/' \
  '/muto_ws/src/muto_description/config/' '/muto_ws/src/muto_description/CMakeLists.txt' \
  '/muto_ws/src/muto_description/package.xml'
git apply -p1 "$D/ros2/bringup.patch"

rm -rf "$OUT"; mkdir -p "$OUT/src"
cp -r muto_ws/src/muto_bringup muto_ws/src/muto_description "$OUT/src/"
cp -r "$D/ros2/muto_policy" "$OUT/src/"
mkdir -p "$OUT/src/muto_description/meshes"
echo "Meshes are not needed on the robot (only for RViz)." > "$OUT/src/muto_description/meshes/README.md"
cp "$D"/muto_walk_policy.onnx "$D"/muto_steps_policy.onnx "$D"/test_vectors.json "$D"/test_vectors_steps.json \
   "$OUT/src/muto_description/config/"
cp -r "$HERE/Dockerfile" "$HERE/tools" "$HERE/video.sh" "$HERE/stop.sh" "$OUT/"
rm -rf "$TMP"
echo "ready: $OUT  (copy it to the robot: scp -r $OUT muto@<robot-ip>:muto_rl)"
