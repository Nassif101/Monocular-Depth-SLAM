from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml

from monocular_depth_slam.datasets.tum import (
    CameraCalibration,
    associate_timestamps,
    load_config,
    load_sequence,
    read_groundtruth,
    read_image_index,
    read_rgb,
    read_sensor_depth,
)


@pytest.fixture
def camera():
    return CameraCalibration(2, 2, 1.0, 1.0, 0.5, 0.5)


@pytest.fixture
def sequence_root(tmp_path):
    root = tmp_path / "sequence"
    (root / "rgb").mkdir(parents=True)
    (root / "depth").mkdir()
    rgb_stamps = ("1.000000", "1.100000", "1.200000")
    depth_stamps = ("1.004000", "1.102000", "1.400000")
    for stamp in rgb_stamps:
        Image.fromarray(np.zeros((2, 2, 3), dtype=np.uint8)).save(root / "rgb" / f"{stamp}.png")
    for stamp in depth_stamps:
        Image.fromarray(np.array([[0, 5000], [10000, 65535]], dtype=np.uint16)).save(
            root / "depth" / f"{stamp}.png")
    (root / "rgb.txt").write_text("# RGB index\n\n" + "\n".join(
        f"{stamp} rgb/{stamp}.png" for stamp in rgb_stamps) + "\n")
    (root / "depth.txt").write_text("\n".join(
        f"{stamp} depth/{stamp}.png" for stamp in depth_stamps) + "\n")
    (root / "groundtruth.txt").write_text(
        "# timestamp tx ty tz qx qy qz qw\n"
        "0.999000 1 2 3 0 0 0 1\n"
        "1.101000 2 3 4 0 0 0 1\n"
        "1.201000 3 4 5 0 0 0 1\n")
    return root


def test_load_preserves_rgb_timeline_and_reports_unmatched_depth(sequence_root, camera):
    sequence = load_sequence(sequence_root, camera)
    assert len(sequence.frames) == 3
    assert sequence.frames[0].rgb.timestamp_text == "1.000000"
    assert sequence.frames[0].depth.timestamp_text == "1.004000"
    assert sequence.frames[2].depth is None
    assert all(frame.pose is not None for frame in sequence.frames)
    summary = sequence.summary()
    assert summary["rgb_depth_matches"] == 2
    assert summary["rgb_without_depth"] == ["1.200000"]
    assert summary["fully_associated_frames"] == 2
    assert summary["pixels_transformed"] is False


def test_pose_mismatch_does_not_drop_rgb_frames(sequence_root, camera):
    sequence = load_sequence(sequence_root, camera, max_pose_difference_s=0.0001)
    assert len(sequence.frames) == 3
    assert all(frame.pose is None for frame in sequence.frames)
    assert len(sequence.summary()["rgb_without_pose"]) == 3


def test_global_closest_association_prevents_early_greedy_mismatch():
    assert associate_timestamps([1.0, 1.1], [1.09], 0.1) == {1: 0}


def test_matches_are_one_to_one_and_ties_are_deterministic():
    assert associate_timestamps([0.0, 2.0], [1.0], 1.0) == {0: 0}
    assert associate_timestamps([1.0], [0.75, 1.25], 0.25) == {0: 0}


def test_inclusive_tolerance_zero_tolerance_and_empty_streams():
    assert associate_timestamps([1.0], [1.02], 0.02) == {0: 0}
    assert associate_timestamps([1.0], [1.0200001], 0.02) == {}
    assert associate_timestamps([1.125], [1.25], 0.125) == {0: 0}
    assert associate_timestamps([1.125], [1.25], 0.124) == {}
    assert associate_timestamps([1.0], [1.0], 0) == {0: 0}
    assert associate_timestamps([], [1.0], 0.02) == {}
    assert associate_timestamps([1.0], [], 0.02) == {}


@pytest.mark.parametrize("tolerance", [-0.1, float("nan"), float("inf")])
def test_invalid_association_tolerance(tolerance):
    with pytest.raises(ValueError, match="tolerance"):
        associate_timestamps([1.0], [1.0], tolerance)


@pytest.mark.parametrize("first,second", [
    ([1.0, 1.0], [1.0]), ([2.0, 1.0], [1.0]), ([1.0], [2.0, 1.0]),
    ([float("nan")], [1.0]), ([1.0], [float("inf")]),
])
def test_invalid_association_timestamps(first, second):
    with pytest.raises(ValueError):
        associate_timestamps(first, second, 0.02)


def test_sensor_depth_units_invalid_pixels_and_no_second_scale_correction(sequence_root, camera):
    sequence = load_sequence(sequence_root, camera)
    depth = read_sensor_depth(sequence.depth_records[0], camera)
    assert depth.metres.dtype == np.float32
    np.testing.assert_array_equal(depth.valid_mask, [[False, True], [True, True]])
    assert np.isnan(depth.metres[0, 0])
    assert depth.metres[0, 1] == 1.0  # not 1.035: TUM already corrected it
    assert depth.metres[1, 0] == 2.0
    assert depth.metres[1, 1] == pytest.approx(65535 / 5000)


def test_rgb_shape_and_channel_order(sequence_root, camera):
    path = sequence_root / "rgb/1.000000.png"
    pixels = np.zeros((2, 2, 3), dtype=np.uint8)
    pixels[0, 0] = [255, 13, 4]
    Image.fromarray(pixels).save(path)
    sequence = load_sequence(sequence_root, camera)
    rgb = read_rgb(sequence.rgb_records[0], camera)
    assert rgb.shape == (2, 2, 3)
    assert rgb.dtype == np.uint8
    np.testing.assert_array_equal(rgb[0, 0], [255, 13, 4])


def test_rejects_wrong_image_size_and_depth_bit_depth(sequence_root, camera):
    sequence = load_sequence(sequence_root, camera)
    Image.fromarray(np.zeros((3, 2, 3), dtype=np.uint8)).save(sequence.rgb_records[0].path)
    with pytest.raises(ValueError, match="image size"):
        read_rgb(sequence.rgb_records[0], camera)
    Image.fromarray(np.zeros((2, 2), dtype=np.uint8)).save(sequence.depth_records[0].path)
    with pytest.raises(ValueError, match="16-bit"):
        read_sensor_depth(sequence.depth_records[0], camera)


@pytest.mark.parametrize("name", ["rgb.txt", "depth.txt", "groundtruth.txt"])
def test_missing_indices_fail_explicitly(sequence_root, camera, name):
    (sequence_root / name).unlink()
    with pytest.raises(FileNotFoundError):
        load_sequence(sequence_root, camera)


def test_missing_image_is_not_silently_dropped(sequence_root, camera):
    (sequence_root / "depth/1.004000.png").unlink()
    with pytest.raises(FileNotFoundError, match="missing image"):
        load_sequence(sequence_root, camera)
    sequence = load_sequence(sequence_root, camera, check_files=False)
    assert len(sequence.frames) == 3


@pytest.mark.parametrize("text", [
    "# comments only\n", "not-a-time rgb/a.png\n", "nan rgb/a.png\n",
    "1.0 rgb/a.png extra\n", "1.0 rgb/a.png\n1.0 rgb/b.png\n",
    "2.0 rgb/a.png\n1.0 rgb/b.png\n", "1.0 ../outside.png\n",
    "1.0 /absolute.png\n",
])
def test_rejects_malformed_or_unsafe_image_indices(tmp_path, text):
    index = tmp_path / "rgb.txt"
    index.write_text(text)
    with pytest.raises(ValueError):
        read_image_index(index, tmp_path, check_files=False)


def test_groundtruth_xyzw_order_is_preserved(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text("# comments\n1.000000 1 2 3 0 0 0.707106781 0.707106781 # pose\n")
    pose, = read_groundtruth(path)
    assert pose.translation == (1.0, 2.0, 3.0)
    assert pose.quaternion_xyzw == (0.0, 0.0, 0.707106781, 0.707106781)
    assert pose.timestamp_text == "1.000000"


@pytest.mark.parametrize("text", [
    "1 0 0 0 0 0 0 0\n", "1 0 0 0 0 0 0 2\n", "1 0 0 0 0 0 1\n",
    "1 nan 0 0 0 0 0 1\n", "2 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n",
])
def test_rejects_invalid_groundtruth(tmp_path, text):
    path = tmp_path / "groundtruth.txt"
    path.write_text(text)
    with pytest.raises(ValueError):
        read_groundtruth(path)


def test_association_export_preserves_timestamp_text(sequence_root, camera, tmp_path):
    sequence = load_sequence(sequence_root, camera)
    output = tmp_path / "associations.txt"
    sequence.write_rgb_depth_associations(output)
    lines = output.read_text().splitlines()
    assert len(lines) == 3
    assert lines[1] == "1.000000 rgb/1.000000.png 1.004000 depth/1.004000.png"
    assert "1.200000" not in output.read_text()


def test_config_calibration_is_explicit_and_root_is_portable(sequence_root, tmp_path):
    config = yaml.safe_load((Path(__file__).parents[1] /
                            "configs/datasets/tum_rgbd.example.yaml").read_text())
    config["root"] = sequence_root.name
    config["camera"].update(width=2, height=2)
    path = tmp_path / "dataset.yaml"
    path.write_text(yaml.safe_dump(config))
    sequence = load_config(path, project_root=tmp_path)
    assert sequence.root == sequence_root
    assert sequence.camera.fx == 517.306408
    assert sequence.camera.depth_units_per_metre == 5000
    assert sequence.camera.distortion == tuple(config["camera"]["distortion"])
    config["camera"]["undistort"] = True
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="undistortion"):
        load_config(path, project_root=tmp_path)


@pytest.mark.parametrize("changes", [{"width": 0}, {"fx": 0}, {"fy": float("nan")},
                                          {"depth_units_per_metre": -1},
                                          {"distortion": (0, 0, 0, 0)}])
def test_invalid_camera_calibration(changes):
    values = dict(width=2, height=2, fx=1.0, fy=1.0, cx=0.5, cy=0.5)
    values.update(changes)
    with pytest.raises(ValueError):
        CameraCalibration(**values)
