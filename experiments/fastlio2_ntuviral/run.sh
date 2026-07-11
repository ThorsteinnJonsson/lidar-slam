#!/usr/bin/env bash
# Build (first time only, ~10-15 min) and run FAST-LIO2 on eee_03.bag inside a
# ROS Noetic container. Records /Odometry to output/fastlio_odom.bag. See
# HANDOFF.md for background and eval.sh for turning the output into a number.
#
# The bag is ~182 s and plays at real-time rate, so a full run takes ~3-4 min
# of wall clock after roslaunch comes up (longer than a default 2-minute Bash
# tool timeout - run this with an extended timeout or in the background).
#
# Usage:
#   ./run.sh             # RViz window shown; run `xhost +local:docker` first
#   ./run.sh --no-rviz   # headless - no X11 needed, use this to validate the
#                         # pipeline before bothering with a display
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

IMAGE=fastlio2-ntuviral
DATASET_DIR="$(cd ../../datasets/ntu_viral/eee_03 && pwd)"
BAG_NAME=eee_03.bag

RVIZ=true
if [[ "${1:-}" == "--no-rviz" ]]; then
  RVIZ=false
fi

if [[ ! -f "$DATASET_DIR/$BAG_NAME" ]]; then
  echo "Expected bag at $DATASET_DIR/$BAG_NAME - not found." >&2
  exit 1
fi

mkdir -p output
rm -f output/fastlio_odom.bag

docker build -t "$IMAGE" .

DOCKER_ARGS=(--rm
  -v "$DATASET_DIR":/data:ro
  -v "$(pwd)/output":/output
)

if [[ "$RVIZ" == true ]]; then
  echo "NOTE: run 'xhost +local:docker' on the host first, or the RViz window will fail to open."
  DOCKER_ARGS+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

docker run "${DOCKER_ARGS[@]}" "$IMAGE" \
  roslaunch /root/catkin_ws/eval_wrapper.launch \
    bag_file:="/data/$BAG_NAME" rviz:="$RVIZ" autorun:=true \
    output_bag:=/output/fastlio_odom.bag

echo "Done. Recorded: output/fastlio_odom.bag"
ls -lh output/fastlio_odom.bag
