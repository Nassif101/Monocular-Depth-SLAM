"""Convert cached canonical DA3 z-depth into aligned nominal-metre ORB input."""
import numpy as np
from PIL import Image

from monocular_depth_slam.depth.metrics import canonical_to_metres, resize_depth_to_reference


def aligned_orb_depth(canonical, processed_intrinsics, original_intrinsics, shape):
    canonical = np.asarray(canonical, dtype=np.float32)
    if canonical.ndim != 2 or len(shape) != 2 or min(shape) <= 0:
        raise ValueError("Expected 2D canonical depth and target dimensions")
    expected = np.asarray(original_intrinsics, dtype=np.float64).copy()
    if expected.shape != (3, 3):
        raise ValueError("Expected original RGB intrinsics")
    expected[0] *= canonical.shape[1]/shape[1]
    expected[1] *= canonical.shape[0]/shape[0]
    if not np.allclose(processed_intrinsics, expected, rtol=1e-6, atol=1e-5):
        raise ValueError("Processed intrinsics do not describe a pure resize of the RGB grid")
    metres = canonical_to_metres(canonical, processed_intrinsics)
    # Conservative invalidation: any nonfinite contribution after resize is zero.
    metres[~np.isfinite(metres) | (metres <= 0)] = np.nan
    depth = resize_depth_to_reference(metres, shape).astype(np.float32)
    depth[~np.isfinite(depth) | (depth <= 0)] = 0
    if not np.any(depth > 0):
        raise ValueError("No valid positive predicted depth remains")
    return depth


def save_float_depth(path, depth):
    depth = np.asarray(depth)
    if depth.ndim != 2 or depth.dtype != np.float32 or not np.isfinite(depth).all() or np.any(depth < 0):
        raise ValueError("ORB depth must be finite nonnegative 2D float32")
    # Single-channel float TIFF retains metre values without integer quantization.
    Image.fromarray(depth).save(path, format="TIFF", compression="raw")
