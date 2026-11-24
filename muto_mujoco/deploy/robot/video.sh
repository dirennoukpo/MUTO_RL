#!/bin/bash
# Runs the sim-video sequence on the robot (the driver + policy must be running in the muto_run container).
docker exec -it muto_run bash -c "source /opt/ros/jazzy/setup.bash && python3 /tools/video_seq.py" 2>&1 | grep -v setlocale
