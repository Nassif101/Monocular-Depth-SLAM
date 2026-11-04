"""TUM RGB-D indices, deterministic timestamp association, and sensor image I/O.

RGB frames remain the primary timeline, including frames without depth or poses.
Ground-truth depth is exposed explicitly as sensor data, never as a prediction.
"""
from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass
from decimal import Decimal
import math
from pathlib import Path, PurePosixPath
from typing import Sequence

import numpy as np
from PIL import Image
import yaml


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)
    depth_units_per_metre: float = 5000.0
    calibration_source: str = "explicit caller calibration"

    def __post_init__(self):
        if (type(self.width) is not int or type(self.height) is not int or
                self.width <= 0 or self.height <= 0):
            raise ValueError("Camera width and height must be positive integers")
        values = (self.fx, self.fy, self.cx, self.cy, self.depth_units_per_metre,
                  *self.distortion)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Camera calibration must contain finite values")
        if self.fx <= 0 or self.fy <= 0 or self.depth_units_per_metre <= 0:
            raise ValueError("Focal lengths and depth units per metre must be positive")
        if len(self.distortion) != 5:
            raise ValueError("Expected distortion in OpenCV order (k1, k2, p1, p2, k3)")
        if not self.calibration_source:
            raise ValueError("Camera calibration requires a source")

    @property
    def matrix(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)


@dataclass(frozen=True)
class ImageRecord:
    timestamp: float
    timestamp_text: str
    relative_path: str
    path: Path


@dataclass(frozen=True)
class PoseRecord:
    timestamp: float
    timestamp_text: str
    translation: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class AssociatedFrame:
    rgb: ImageRecord
    depth: ImageRecord | None
    pose: PoseRecord | None


@dataclass(frozen=True)
class SensorDepth:
    metres: np.ndarray
    valid_mask: np.ndarray


def _data_lines(path):
    with Path(path).open(encoding="utf-8-sig") as stream:
        for number, line in enumerate(stream, 1):
            content = line.partition("#")[0].strip()
            if content:
                yield number, content.split()


def _number(text, location):
    try:
        value = float(text)
    except ValueError as error:
        raise ValueError(f"{location}: invalid number {text!r}") from error
    if not math.isfinite(value):
        raise ValueError(f"{location}: non-finite number {text!r}")
    return value


def _check_timestamp_order(timestamp, previous, location):
    if previous is not None and timestamp <= previous:
        raise ValueError(f"{location}: timestamps must be strictly increasing (no duplicates)")


def read_image_index(path, root, *, check_files=True):
    """Read timestamp/path pairs without rounding timestamp strings."""
    root = Path(root).resolve()
    records = []
    previous = None
    for line, fields in _data_lines(path):
        location = f"{path}:{line}"
        if len(fields) != 2:
            raise ValueError(f"{location}: expected timestamp and relative image path")
        timestamp = _number(fields[0], location)
        _check_timestamp_order(timestamp, previous, location)
        relative = PurePosixPath(fields[1])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"{location}: image path must stay within the sequence")
        image_path = (root / relative).resolve()
        if not image_path.is_relative_to(root):
            raise ValueError(f"{location}: image path resolves outside the sequence")
        if check_files and not image_path.is_file():
            raise FileNotFoundError(f"{location}: missing image: {image_path}")
        records.append(ImageRecord(timestamp, fields[0], fields[1], image_path))
        previous = timestamp
    if not records:
        raise ValueError(f"{path}: no image records")
    return tuple(records)


def read_groundtruth(path):
    """Read camera-to-world translation and quaternion in TUM's xyzw order."""
    records = []
    previous = None
    for line, fields in _data_lines(path):
        location = f"{path}:{line}"
        if len(fields) != 8:
            raise ValueError(f"{location}: expected timestamp tx ty tz qx qy qz qw")
        values = tuple(_number(value, location) for value in fields)
        timestamp = values[0]
        _check_timestamp_order(timestamp, previous, location)
        quaternion = values[4:8]
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isclose(norm, 1.0, abs_tol=1e-3, rel_tol=0):
            raise ValueError(f"{location}: expected a unit quaternion, got norm {norm}")
        records.append(PoseRecord(timestamp, fields[0], values[1:4], quaternion))
        previous = timestamp
    if not records:
        raise ValueError(f"{path}: no ground-truth poses")
    return tuple(records)


def associate_timestamps(first: Sequence[float], second: Sequence[float], max_difference: float):
    """Return one-to-one matches, taking globally closest candidate pairs first.

    Matches include the tolerance boundary. Ties prefer earlier first timestamps,
    then earlier second timestamps. This is deterministic greedy matching, not an
    interpolation or a maximum-cardinality assignment.
    """
    if not math.isfinite(max_difference) or max_difference < 0:
        raise ValueError("Timestamp tolerance must be finite and non-negative")
    for timestamps in (first, second):
        previous = None
        for timestamp in timestamps:
            if not math.isfinite(timestamp):
                raise ValueError("Timestamps must be finite")
            _check_timestamp_order(timestamp, previous, "association")
            previous = timestamp
    candidates = []
    # Decimal comparisons keep an exact decimal boundary such as 1.02 - 1.00
    # from being rejected by binary floating-point subtraction at a 0.02 limit.
    first_decimal = [Decimal(str(timestamp)) for timestamp in first]
    second_decimal = [Decimal(str(timestamp)) for timestamp in second]
    tolerance = Decimal(str(max_difference))
    for left, timestamp in enumerate(first_decimal):
        lower = bisect_left(second_decimal, timestamp - tolerance)
        upper = bisect_right(second_decimal, timestamp + tolerance)
        for right in range(lower, upper):
            difference = abs(timestamp - second_decimal[right])
            if difference <= tolerance:
                candidates.append((difference, left, right))
    used_left, used_right, matches = set(), set(), {}
    for _, left, right in sorted(candidates):
        if left not in used_left and right not in used_right:
            matches[left] = right
            used_left.add(left)
            used_right.add(right)
    return matches


def _check_image_size(image, camera, path):
    if image.size != (camera.width, camera.height):
        raise ValueError(f"{path}: image size {image.size} does not match camera "
                         f"{(camera.width, camera.height)}")


def read_rgb(record: ImageRecord, camera: CameraCalibration):
    """Return uint8 RGB pixels without undistortion, resizing or cropping."""
    with Image.open(record.path) as image:
        _check_image_size(image, camera, record.path)
        if image.format != "PNG" or image.mode != "RGB":
            raise ValueError(f"{record.path}: expected an 8-bit RGB PNG")
        return np.array(image, dtype=np.uint8)


def read_sensor_depth(record: ImageRecord, camera: CameraCalibration):
    """Convert sensor PNG values to z-depth in metres; zero pixels become NaN.

    TUM already applied the Freiburg depth correction factors. Do not multiply
    these values by 1.035/1.031 a second time.
    """
    with Image.open(record.path) as image:
        _check_image_size(image, camera, record.path)
        if image.format != "PNG" or image.mode not in ("I;16", "I;16L", "I;16B", "I"):
            raise ValueError(f"{record.path}: expected a 16-bit monochrome depth PNG")
        raw = np.array(image)
    if raw.ndim != 2 or raw.dtype.kind not in "iu" or raw.min() < 0 or raw.max() > 65535:
        raise ValueError(f"{record.path}: invalid unsigned 16-bit sensor depth")
    valid = raw != 0
    metres = raw.astype(np.float32) / np.float32(camera.depth_units_per_metre)
    metres[~valid] = np.nan
    return SensorDepth(metres, valid)


@dataclass(frozen=True)
class TUMSequence:
    root: Path
    camera: CameraCalibration
    rgb_records: tuple[ImageRecord, ...]
    depth_records: tuple[ImageRecord, ...]
    poses: tuple[PoseRecord, ...]
    frames: tuple[AssociatedFrame, ...]
    max_depth_difference_s: float
    max_pose_difference_s: float

    def summary(self):
        return {
            "sequence": self.root.name,
            "rgb_frames": len(self.rgb_records),
            "depth_frames": len(self.depth_records),
            "groundtruth_poses": len(self.poses),
            "rgb_depth_matches": sum(frame.depth is not None for frame in self.frames),
            "rgb_pose_matches": sum(frame.pose is not None for frame in self.frames),
            "fully_associated_frames": sum(frame.depth is not None and frame.pose is not None
                                           for frame in self.frames),
            "rgb_without_depth": [frame.rgb.timestamp_text for frame in self.frames
                                  if frame.depth is None],
            "rgb_without_pose": [frame.rgb.timestamp_text for frame in self.frames
                                 if frame.pose is None],
            "max_depth_difference_s": self.max_depth_difference_s,
            "max_pose_difference_s": self.max_pose_difference_s,
            "association": "global_closest_first_one_to_one_inclusive_tolerance",
            "groundtruth_interpolation": "none",
            "camera": asdict(self.camera),
            "pixels_transformed": False,
            "sensor_depth_convention": "z_depth_metres; raw / depth_units_per_metre",
        }

    def write_rgb_depth_associations(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as stream:
            stream.write("# rgb_timestamp rgb_path depth_timestamp depth_path\n")
            for frame in self.frames:
                if frame.depth is not None:
                    stream.write(f"{frame.rgb.timestamp_text} {frame.rgb.relative_path} "
                                 f"{frame.depth.timestamp_text} {frame.depth.relative_path}\n")


def load_sequence(root, camera, *, rgb_index="rgb.txt", depth_index="depth.txt",
                  groundtruth_index="groundtruth.txt", max_depth_difference_s=0.02,
                  max_pose_difference_s=0.02, check_files=True):
    root = Path(root).resolve()
    rgb = read_image_index(root / rgb_index, root, check_files=check_files)
    depth = read_image_index(root / depth_index, root, check_files=check_files)
    poses = read_groundtruth(root / groundtruth_index)
    rgb_times = [record.timestamp for record in rgb]
    depth_matches = associate_timestamps(rgb_times, [record.timestamp for record in depth],
                                         max_depth_difference_s)
    pose_matches = associate_timestamps(rgb_times, [pose.timestamp for pose in poses],
                                        max_pose_difference_s)
    frames = tuple(AssociatedFrame(record,
                                   depth[depth_matches[i]] if i in depth_matches else None,
                                   poses[pose_matches[i]] if i in pose_matches else None)
                   for i, record in enumerate(rgb))
    return TUMSequence(root, camera, rgb, depth, poses, frames,
                       max_depth_difference_s, max_pose_difference_s)


def load_config(config_path, *, project_root=None, check_files=True):
    """Load explicit camera settings; relative dataset roots use project_root/cwd."""
    config = yaml.safe_load(Path(config_path).read_text())
    camera = config["camera"]
    if camera.get("undistort", False):
        raise ValueError("This loader preserves registered pixels; undistortion is not implemented")
    if camera.get("depth_registered_to_rgb") is not True:
        raise ValueError("This loader requires depth already registered to RGB")
    if camera.get("distortion_model") != "opencv_radial_tangential":
        raise ValueError("Unsupported camera distortion model")
    calibration = CameraCalibration(
        **{key: camera[key] for key in ("width", "height", "fx", "fy", "cx", "cy",
                                        "depth_units_per_metre", "calibration_source")},
        distortion=tuple(camera["distortion"]),
    )
    root = Path(project_root or Path.cwd()) / config["root"]
    return load_sequence(root, calibration, check_files=check_files,
                         **{key: config[key] for key in ("rgb_index", "depth_index",
                                                         "groundtruth_index",
                                                         "max_depth_difference_s",
                                                         "max_pose_difference_s")})
