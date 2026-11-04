import pytest
from monocular_depth_slam.manifest import validate_manifest


def test_valid_manifest():
    validate_manifest(dict(dataset="tum_rgbd", sequence="fr1_xyz", slam_revision="pin-me",
                           depth_revision="pin-me", alignment="se3"))


def test_rejects_missing_revision():
    with pytest.raises(ValueError):
        validate_manifest(dict(dataset="tum_rgbd", sequence="fr1_xyz",
                               slam_revision="pin-me", alignment="sim3"))
