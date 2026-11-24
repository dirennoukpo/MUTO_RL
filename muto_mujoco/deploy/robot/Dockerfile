# Robot image for the MUTO RS policy: ROS 2 Jazzy + ros2_control + ONNX Runtime (Pi 5, arm64).
# Based on Diren's Dockerfile.muto_rs, without the Gazebo simulator packages. ONNX Runtime comes from the
# ROS package ros-jazzy-onnxruntime-vendor (the GitHub release download returned 404 on 2026-11-21).
# Build context: the folder made by assemble.sh (src/ = muto_bringup + muto_description + muto_policy).
FROM ros:jazzy-ros-base
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git wget ca-certificates nlohmann-json3-dev python3-colcon-common-extensions \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager ros-jazzy-hardware-interface \
      ros-jazzy-joint-state-broadcaster ros-jazzy-position-controllers ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher ros-jazzy-teleop-twist-keyboard ros-jazzy-onnxruntime-vendor \
    && rm -rf /var/lib/apt/lists/*
ENV ORT=/opt/ros/jazzy/opt/onnxruntime_vendor
RUN cd $ORT/lib && [ -e libonnxruntime.so ] || ln -s libonnxruntime.so.* libonnxruntime.so
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/opt/onnxruntime_vendor/lib
COPY src /muto_ws/src
WORKDIR /muto_ws
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DONNXRUNTIME_ROOT=$ORT"
RUN echo "source /opt/ros/jazzy/setup.bash && source /muto_ws/install/setup.bash" >> /root/.bashrc
CMD ["bash"]
