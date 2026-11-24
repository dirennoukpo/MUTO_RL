#!/bin/bash
# Stops the running movement sequence. The policy keeps running and goes back to standing still within 0.5 s.
docker exec muto_run pkill -f video_seq.py && echo "sequence stopped, robot standing" || echo "no sequence running"
