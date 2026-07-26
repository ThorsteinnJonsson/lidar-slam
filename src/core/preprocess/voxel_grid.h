#pragma once

#include "types.h"

// Voxel-grid downsampling. Overlays a regular grid of cubes (side = leaf_size,
// meters) and replaces all points in each occupied voxel with their centroid.
// Output keeps x/y/z and intensity (averaged); ring and t_offset_ns are dropped
// (they lose meaning once points are merged). Scan-level stamp/frame_id carry
// over. Non-finite points are skipped.
//
// leaf_size must be > 0.
PointCloud voxel_downsample(const PointCloud& cloud, double leaf_size);
