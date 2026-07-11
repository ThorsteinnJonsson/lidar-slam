#!/usr/bin/env bash
# Headless visual check: runs FAST-LIO2 + RViz inside a virtual X display
# (Xvfb, no host X server needed) and screenshots it every 5s to
# output/screenshots/, so the ground-plane-level question in HANDOFF.md can be
# checked from images instead of requiring someone to watch a live window.
# Takes ~3-4 min (same real-time bag playback as run.sh).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

IMAGE=fastlio2-ntuviral
DATASET_DIR="$(cd ../../datasets/ntu_viral/eee_03 && pwd)"
BAG_NAME=eee_03.bag

mkdir -p output/screenshots
rm -f output/screenshots/*.png output/fastlio_odom_capture.bag

docker build -t "$IMAGE" .

docker run --rm \
  -v "$DATASET_DIR":/data:ro \
  -v "$(pwd)/output":/output \
  -e BAG_FILE="/data/$BAG_NAME" \
  "$IMAGE" /root/catkin_ws/capture_entrypoint.sh

echo "Screenshots:"
ls -la output/screenshots/
