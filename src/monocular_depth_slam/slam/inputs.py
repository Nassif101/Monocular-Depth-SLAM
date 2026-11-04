"""Select submitted RGB frames while retaining the loader's fixed associations."""


def selected_frames(sequence, protocol):
    selection = protocol.get("frame_selection", "all_rgb")
    if selection == "all_rgb":
        frames = sequence.frames
    elif selection == "rgb_with_sensor_depth_match":
        frames = tuple(frame for frame in sequence.frames if frame.depth is not None)
    else:
        raise ValueError(f"Unsupported frame selection: {selection}")
    if not frames:
        raise ValueError("No RGB frames selected")
    return frames


def validate_submitted_timeline(frames, manifest):
    expected = [(frame.rgb.timestamp_text, frame.rgb.relative_path) for frame in frames]
    recorded = [(row["timestamp"], row["path"]) for row in manifest["rgb_inputs"]]
    if expected != recorded:
        raise ValueError("Selected input timeline differs from the run manifest")
