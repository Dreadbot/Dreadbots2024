#!/bin/bash

v4l2-ctl -d /dev/video0 -c auto_exposure=1
v4l2-ctl -d /dev/video0 -c exposure_time_absolute=10,brightness=300
cd /home/vision
python3 norm_detector.py intrinsics.yaml
