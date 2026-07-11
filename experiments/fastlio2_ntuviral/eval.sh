#!/usr/bin/env bash
# Convert the recorded FAST-LIO2 odometry bag to TUM and evaluate against our
# ground truth. Runs on the HOST (evo reads bags directly, no ROS needed) - do
# not run this inside the container. Requires ./run.sh to have completed.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

EVO_BIN="$HOME/.venvs/evo/bin"
ODOM_BAG="output/fastlio_odom.bag"
GT="../../evaluation/gt.tum"

if [[ ! -f "$ODOM_BAG" ]]; then
  echo "Missing $ODOM_BAG - run ./run.sh first." >&2
  exit 1
fi
if [[ ! -f "$GT" ]]; then
  echo "Missing $GT - run the main lidar_slam pipeline at least once so it" >&2
  echo "writes evaluation/gt.tum from the leica ground-truth topic." >&2
  exit 1
fi
if [[ ! -x "$EVO_BIN/evo_ape" ]]; then
  echo "evo not found at $EVO_BIN - adjust EVO_BIN in this script." >&2
  exit 1
fi

# evo's exact output filename for --save_as_tum varies by version, so convert
# in a scratch dir and pick up whatever .tum file appears rather than guessing
# the name.
rm -rf output/evo_scratch
mkdir -p output/evo_scratch
cp "$ODOM_BAG" output/evo_scratch/
(
  cd output/evo_scratch
  "$EVO_BIN/evo_traj" bag "$(basename "$ODOM_BAG")" /Odometry --save_as_tum
)
GENERATED_TUM=$(ls -t output/evo_scratch/*.tum 2>/dev/null | head -n1)
if [[ -z "$GENERATED_TUM" ]]; then
  echo "evo_traj did not produce a .tum file - check its output above." >&2
  exit 1
fi
cp "$GENERATED_TUM" output/fastlio_traj.tum
rm -rf output/evo_scratch

echo "=== evo_ape (aligned, translation part) ==="
"$EVO_BIN/evo_ape" tum "$GT" output/fastlio_traj.tum \
  --align -r trans_part --t_max_diff 0.03 -va \
  | tee output/evo_ape_output.txt

python3 - << 'PYEOF'
import re, math, pathlib

text = pathlib.Path("output/evo_ape_output.txt").read_text()
m = re.search(
    r"Rotation of alignment:\s*\[\[(.+?)\]\s*\[(.+?)\]\s*\[(.+?)\]\]",
    text, re.S,
)
if not m:
    raise SystemExit("Could not find 'Rotation of alignment' in evo output "
                      "(pass -v/-va to evo_ape).")
row2 = [float(x) for x in m.group(3).split()]
r22 = max(-1.0, min(1.0, row2[2]))

# FAST-LIO2 (IKFoM build) seeds attitude at IDENTITY and puts "which way is up"
# entirely into a free gravity state (see IMU_Processing.hpp: IMU_init() sets
# init_state.grav = S2(-mean_acc/|mean_acc| * G) but never touches rot_end).
# Its world Z axis is therefore whatever the IMU's body Z happened to point at
# t=0, NOT physical up - confirmed empirically: raw z climbs to -3.5..-3.9 m in
# fastlio_traj.tum during the same window GT climbs to +3.4..+4.2 m. Evo's
# Umeyama fit correctly finds this as a real ~180 deg rotation (dominated by a
# sign flip on R[2][2]), which drowns out the actual few-degree tilt we care
# about. Un-flip it before reading off the tilt: R[2][2] of a matrix is
# invariant to composing an extra yaw (rotation about Z) on either side, and
# empirically flips sign consistently under the observed convention mismatch
# (verified against early/late trajectory halves independently - see
# RESULT.md). Do not apply this correction to a rotation that ISN'T already
# close to +/-1 in this cell; that would silently hide a real problem instead
# of fixing a known convention mismatch.
r22_corrected = -r22
tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, r22_corrected))))
print(f"\nRaw R[2][2] = {r22:.5f} (dominated by FAST-LIO2's world-frame Z-axis "
      f"convention, not a real ~180 deg attitude error - see comment in this "
      f"script and RESULT.md)")
print(f"Convention-corrected tilt vs ground-truth vertical: {tilt_deg:.2f} deg")
PYEOF
