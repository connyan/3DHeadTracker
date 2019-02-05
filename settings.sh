#!/bin/bash
v4l2-ctl -d /dev/video0 -c power_line_frequency=1
v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=1
#v4l2-ctl -d /dev/video0 -c white_balance_temperature=2000
v4l2-ctl -d /dev/video0 -c backlight_compensation=0
v4l2-ctl -d /dev/video0 -c saturation=100
v4l2-ctl -d /dev/video0 -c brightness=128
v4l2-ctl -d /dev/video0 -c contrast=128
v4l2-ctl -d /dev/video0 -c sharpness=50
v4l2-ctl -d /dev/video0 -c gain=100
v4l2-ctl -d /dev/video0 -c focus_auto=0
v4l2-ctl -d /dev/video0 -c focus_absolute=0
v4l2-ctl -d /dev/video0 -c exposure_auto=1
#outdoors
#v4l2-ctl -d /dev/video0 -c exposure_absolute=10
#indoors
v4l2-ctl -d /dev/video0 -c exposure_absolute=500

v4l2-ctl -d /dev/video1 -c power_line_frequency=1
v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=1
#v4l2-ctl -d /dev/video1 -c white_balance_temperature=1500
v4l2-ctl -d /dev/video1 -c backlight_compensation=0
v4l2-ctl -d /dev/video1 -c saturation=130
v4l2-ctl -d /dev/video1 -c brightness=128
v4l2-ctl -d /dev/video1 -c contrast=128
v4l2-ctl -d /dev/video1 -c sharpness=100
v4l2-ctl -d /dev/video1 -c gain=100
v4l2-ctl -d /dev/video1 -c focus_auto=0
v4l2-ctl -d /dev/video1 -c focus_absolute=0
v4l2-ctl -d /dev/video1 -c exposure_auto=1
#outdoors
#v4l2-ctl -d /dev/video1 -c exposure_absolute=10
#indoors
v4l2-ctl -d /dev/video1 -c exposure_absolute=5000
