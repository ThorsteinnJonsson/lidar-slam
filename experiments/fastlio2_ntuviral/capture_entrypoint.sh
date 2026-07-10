#!/usr/bin/env bash
# Runs INSIDE the container. Starts a virtual X display so RViz can run
# headlessly, launches FAST-LIO2 exactly like eval_wrapper.launch does, and
# periodically screenshots the virtual screen for later visual inspection -
# this lets the agent (not just a human at the keyboard) check whether the
# map/ground stays level, without needing the host's real X server.
set -uo pipefail

mkdir -p /output/screenshots

Xvfb :99 -screen 0 1280x800x24 &
XVFB_PID=$!
export DISPLAY=:99
sleep 2

roslaunch /root/catkin_ws/eval_wrapper.launch \
  bag_file:="${BAG_FILE:-/data/eee_03.bag}" rviz:=true autorun:=true \
  output_bag:=/output/fastlio_odom_capture.bag &
ROSLAUNCH_PID=$!

# Let roscore/rviz/laserMapping come up before the (internally 1s-delayed)
# rosbag player starts.
sleep 8

i=0
while kill -0 "$ROSLAUNCH_PID" 2>/dev/null; do
  ts=$(date +%s)
  import -window root "/output/screenshots/shot_$(printf '%03d' "$i")_${ts}.png" 2>/dev/null || true
  i=$((i + 1))
  sleep 5
done

wait "$ROSLAUNCH_PID" 2>/dev/null
kill "$XVFB_PID" 2>/dev/null || true
echo "Capture done: $i screenshots in /output/screenshots"
