from pathlib import Path
from types import SimpleNamespace

import pytest

from monocular_depth_slam.datasets.tum import AssociatedFrame, ImageRecord, PoseRecord
from monocular_depth_slam.slam.inputs import selected_frames, validate_submitted_timeline
from monocular_depth_slam.slam.trajectory import time_interval_pairs


def test_sensor_subset_preserves_fixed_reference_matches_and_exact_rgb_stamps():
    images = [ImageRecord(i, f"{i}.000000", f"rgb/{i}.png", Path(f"rgb/{i}.png")) for i in range(3)]
    reference = PoseRecord(2.001, "2.001", (0,0,0), (0,0,0,1))
    frames = (AssociatedFrame(images[0], images[0], None),
              AssociatedFrame(images[1], None, None),
              AssociatedFrame(images[2], images[2], reference))
    sequence = SimpleNamespace(frames=frames)
    subset = selected_frames(sequence, {"frame_selection": "rgb_with_sensor_depth_match"})
    assert subset == (frames[0], frames[2])
    assert subset[1].pose is reference
    assert selected_frames(sequence, {}) is frames
    with pytest.raises(ValueError, match="Unsupported"):
        selected_frames(sequence, {"frame_selection": "unknown"})


def test_replaying_subset_keeps_intentional_omissions_separate_from_pose_gaps():
    # RGB source index 1 is intentionally omitted from every method's input.
    times = [0, .6, 1]
    assert time_interval_pairs(times, [0,1,2]) == [(0,2)]
    # A missing pose inside the submitted timeline excludes the same RPE pair.
    assert time_interval_pairs([0,1], [0,2]) == []


def test_manifest_cannot_silently_use_all_frames_for_a_subset_run():
    image = ImageRecord(1.0, "1.000000", "rgb/1.png", Path("rgb/1.png"))
    frame = AssociatedFrame(image, None, None)
    validate_submitted_timeline([frame], {"rgb_inputs": [{"timestamp": "1.000000", "path": "rgb/1.png"}]})
    with pytest.raises(ValueError, match="timeline"):
        validate_submitted_timeline([frame], {"rgb_inputs": []})
