"""Export conversion and time-pair selection with explicit tracking gaps."""
from bisect import bisect_left
from decimal import Decimal
import math
from pathlib import Path


def convert_orb_export(source, destination, rgb_records, allowed_timestamps=None):
    """Convert ORB's space-delimited ns/xyz/xyzw export to canonical RGB stamps.

    ORB's EuRoC-named exporter uses xyzw and spaces, unlike the EuRoC CSV format.
    Restore original RGB timestamp strings after checking the ns rounding error.
    """
    if not rgb_records:
        raise ValueError("An input RGB timeline is required")
    times = [Decimal(record.timestamp_text) for record in rgb_records]
    output, previous = [], None
    with Path(source).open() as stream:
        for number, line in enumerate(stream, 1):
            fields = line.split()
            if not fields or fields[0].startswith("#"):
                continue
            if len(fields) != 8:
                raise ValueError(f"{source}:{number}: expected ns tx ty tz qx qy qz qw")
            timestamp = Decimal(fields[0]) / Decimal(1_000_000_000)
            if not timestamp.is_finite():
                raise ValueError("Non-finite exported timestamp")
            index = bisect_left(times, timestamp)
            candidates = [j for j in (index-1, index) if 0 <= j < len(times)]
            closest = min(candidates, key=lambda j: abs(times[j]-timestamp))
            if abs(times[closest]-timestamp) > Decimal("0.000001"):
                raise ValueError(f"Export timestamp is not an input RGB timestamp: {fields[0]}")
            stamp = rgb_records[closest].timestamp_text
            if previous is not None and times[closest] <= previous:
                raise ValueError("Duplicate or unordered exported timestamps")
            previous = times[closest]
            values = [float(value) for value in fields[1:]]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("Non-finite exported pose")
            norm = math.sqrt(sum(value**2 for value in values[3:]))
            if not math.isclose(norm, 1.0, abs_tol=1e-3, rel_tol=0):
                raise ValueError("Exported quaternion is not unit length")
            if allowed_timestamps is None or stamp in allowed_timestamps:
                output.append(stamp + " " + " ".join(fields[1:]) + "\n")
    if not output:
        raise ValueError("No eligible exported poses")
    Path(destination).write_text("# timestamp tx ty tz qx qy qz qw; camera-to-world\n" + "".join(output))
    return len(output)


def time_interval_pairs(timestamps, input_indices, interval=1.0, tolerance=0.02):
    """One closest future pose per anchor; never bridge missing input frames."""
    if (len(timestamps) != len(input_indices) or not math.isfinite(interval) or
            not math.isfinite(tolerance) or interval <= 0 or tolerance < 0):
        raise ValueError("Invalid time-pair parameters")
    segments = []
    segment = 0
    for i in range(len(timestamps)):
        if not math.isfinite(timestamps[i]):
            raise ValueError("Timestamps must be finite")
        if i:
            if timestamps[i] <= timestamps[i-1] or input_indices[i] <= input_indices[i-1]:
                raise ValueError("Timestamps/input indices must be strictly increasing")
            if input_indices[i] != input_indices[i-1]+1:
                segment += 1
        segments.append(segment)
    pairs = []
    times = [Decimal(str(timestamp)) for timestamp in timestamps]
    for i, timestamp in enumerate(times):
        target = timestamp + Decimal(str(interval))
        index = bisect_left(times, target, lo=i+1)
        candidates = [j for j in (index-1, index) if i < j < len(timestamps)
                      and segments[j] == segments[i]]
        if candidates:
            j = min(candidates, key=lambda candidate: abs(times[candidate]-target))
            if abs(times[j]-target) <= Decimal(str(tolerance)):
                pairs.append((i, j))
    return pairs
