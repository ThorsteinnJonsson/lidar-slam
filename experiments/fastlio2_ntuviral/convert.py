#!/usr/bin/env python3

import sys
import numpy as np
from scipy.spatial.transform import Rotation

## Run this file to convert the output of FAST_LIO2 (TUM) from the IMU-frame to the PRISM-frame
## so we can compare directly to the ground truth.

# Translation from IMU to prism
T = np.array([-0.293656, -0.012288, -0.273095])


def main(input_file, output_file):
    with open(input_file, "r") as fin, open(output_file, "w") as fout:
        for line in fin:
            line = line.strip()

            if not line or line.startswith("#"):
                fout.write(line + "\n")
                continue

            fields = line.split()

            if len(fields) != 8:
                raise ValueError(f"Expected 8 fields, got {len(fields)}:\n{line}")

            ts = fields[0]
            x, y, z = map(float, fields[1:4])
            qx, qy, qz, qw = map(float, fields[4:8])

            p = np.array([x, y, z])

            R = Rotation.from_quat([qx, qy, qz, qw])
            p_new = p + R.apply(T)

            fout.write(
                f"{ts} "
                f"{p_new[0]:.9f} {p_new[1]:.9f} {p_new[2]:.9f} "
                f"{0} {0} {0} {1}\n"
            )


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} fastlio_traj.tum fastlio_traj_prism.tum")
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])