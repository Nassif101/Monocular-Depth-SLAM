"""Explicit temporal calibration boundaries and uniform depth correction."""
import math

import numpy as np


def temporal_partition(frames, prefix_count):
    if type(prefix_count) is not int or not 0 < prefix_count < len(frames):
        raise ValueError("A nonempty calibration prefix and later evaluation split are required")
    prefix, later = tuple(frames[:prefix_count]), tuple(frames[prefix_count:])
    if prefix[-1].rgb.timestamp >= later[0].rgb.timestamp:
        raise ValueError("Temporal calibration/evaluation frames must be strictly ordered")
    return prefix, later


def scale_prepared_depth(depth, scale):
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("Depth correction must be finite and positive")
    depth = np.asarray(depth)
    if depth.ndim != 2 or depth.dtype != np.float32 or not np.isfinite(depth).all() or np.any(depth < 0):
        raise ValueError("Expected finite nonnegative 2D float32 prepared depth")
    result = depth*np.float32(scale)
    if not np.isfinite(result).all():
        raise ValueError("Depth correction overflowed")
    return result
