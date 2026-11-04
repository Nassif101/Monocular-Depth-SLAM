import numpy as np
from PIL import Image
import pytest

from monocular_depth_slam.depth.orb_adapter import aligned_orb_depth, save_float_depth


def test_fixed_conversion_and_float_tiff_preserve_metres(tmp_path):
    original = np.array([[600,0,2],[0,600,2],[0,0,1]], dtype=float)
    processed = np.array([[300,0,1],[0,300,1],[0,0,1]], dtype=float)
    depth = aligned_orb_depth(np.full((2,2), 1.234567, np.float32), processed, original, (4,4))
    np.testing.assert_allclose(depth, 1.234567)
    output = tmp_path/'depth.tiff'
    save_float_depth(output, depth)
    decoded = np.asarray(Image.open(output))
    assert decoded.dtype == np.float32
    np.testing.assert_array_equal(decoded, depth)


def test_invalid_depth_is_zero_without_reference_mask_or_range_clipping():
    k = np.array([[300,0,1],[0,300,1],[0,0,1]], dtype=float)
    depth = aligned_orb_depth([[1,np.nan],[-1,100]], k, k, (2,2))
    np.testing.assert_array_equal(depth, [[1,0],[0,100]])
    with pytest.raises(ValueError, match='No valid'):
        aligned_orb_depth(np.zeros((2,2)), k, k, (2,2))


def test_cropped_or_misaligned_processed_intrinsics_fail():
    k = np.array([[300,0,1],[0,300,1],[0,0,1]], dtype=float)
    cropped = k.copy()
    cropped[0,2] += 1
    with pytest.raises(ValueError, match='pure resize'):
        aligned_orb_depth(np.ones((2,2)), cropped, k, (2,2))
