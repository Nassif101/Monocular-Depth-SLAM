from pathlib import Path

import pytest

from monocular_depth_slam.datasets.tum import ImageRecord
from monocular_depth_slam.slam.trajectory import convert_orb_export, time_interval_pairs


def test_ns_export_restores_exact_rgb_stamp_and_preserves_xyzw(tmp_path):
    stamp = "1305031102.175304"
    frame = ImageRecord(float(stamp), stamp, "rgb/image.png", Path("rgb/image.png"))
    source, destination = tmp_path/"ns.txt", tmp_path/"tum.txt"
    source.write_text("1305031102175303936.000000 1 2 3 0 0 0.6 0.8\n")
    assert convert_orb_export(source, destination, [frame]) == 1
    assert destination.read_text().splitlines()[1] == stamp + " 1 2 3 0 0 0.6 0.8"


def test_nontracked_export_rows_are_excluded_but_input_stamps_retained(tmp_path):
    frames = [ImageRecord(float(i), f"{i}.000000", f"rgb/{i}.png", tmp_path/f"{i}.png")
              for i in (1, 2)]
    source, destination = tmp_path/"ns.txt", tmp_path/"tum.txt"
    source.write_text("1000000000 0 0 0 0 0 0 1\n2000000000 1 0 0 0 0 0 1\n")
    assert convert_orb_export(source, destination, frames, {"2.000000"}) == 1
    assert destination.read_text().splitlines()[1].startswith("2.000000 ")


@pytest.mark.parametrize("row", ["1000000000 0 0 0 0 0 0 0\n",
                                  "1000000000 nan 0 0 0 0 0 1\n",
                                  "10000001000 0 0 0 0 0 0 1\n"])
def test_bad_exports_fail_instead_of_creating_trajectory(tmp_path, row):
    frame = ImageRecord(1.0, "1.000000", "rgb/1.png", tmp_path/"1.png")
    source = tmp_path/"ns.txt"
    source.write_text(row)
    with pytest.raises(ValueError):
        convert_orb_export(source, tmp_path/"tum.txt", [frame])


def test_time_pairs_never_bridge_missing_rgb_inputs():
    pairs = time_interval_pairs([0, .5, 1, 2.5, 3, 3.5], [0, 1, 2, 5, 6, 7])
    assert pairs == [(0, 2), (3, 5)]


def test_time_pair_decimal_boundary_and_earlier_tie():
    assert time_interval_pairs([0, 1.02], [0, 1], 1, .02) == [(0, 1)]
    assert time_interval_pairs([0, .99, 1.01], [0, 1, 2], 1, .02) == [(0, 1)]


def test_nonfinite_interval_is_rejected():
    with pytest.raises(ValueError):
        time_interval_pairs([0, 1], [0, 1], float("nan"))
