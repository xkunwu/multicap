#!/bin/bash

# ./calib-multi_cameras_calibration -nc 5 -pw 1600 -ph 1200 -ct 0 -fe 0 -nm 25 -v 1 /home/xwu/projects/multicap/data/demo/multi_camera_omnidir.xml

# for f in *.png; do mv $f $(echo $f | awk 'sub(/[0-9]/, $0+1)'); done

# /home/xwu/projects/opencv-samples/build/cpp/cpp-example-imagelist_creator /home/xwu/projects/multicap/data/check/camera_calibration_check.xml /home/xwu/projects/multicap/data/check/*.jpg

/home/xwu/projects/opencv-samples/build/cpp/cpp-example-imagelist_creator /home/xwu/projects/multicap/data/demo/multi_camera_omnidir.xml /home/xwu/projects/multicap/data/demo/images/*.png

