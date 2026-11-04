"""evo-backed alignment and pose errors, plus explicit coverage/scale diagnostics."""
import numpy as np
from evo.core import geometry, lie_algebra, metrics
from evo.core.trajectory import PoseTrajectory3D


def as_evo(records):
    positions = np.array([record.translation for record in records], dtype=np.float64)
    xyzw = np.array([record.quaternion_xyzw for record in records], dtype=np.float64)
    return PoseTrajectory3D(positions_xyz=positions,
                            orientations_quat_wxyz=xyzw[:, [3, 0, 1, 2]],
                            timestamps=np.array([record.timestamp for record in records]))


def statistics(values):
    values = np.asarray(values, dtype=np.float64)
    if not values.size:
        return {"count": 0, "rmse": None, "mean": None, "median": None,
                "std": None, "min": None, "max": None}
    if not np.isfinite(values).all():
        raise ValueError("Metric contains non-finite errors")
    return {"count": len(values), "rmse": float(np.sqrt(np.mean(values**2))),
            "mean": float(np.mean(values)), "median": float(np.median(values)),
            "std": float(np.std(values)), "min": float(values.min()), "max": float(values.max())}


def aligned_errors(reference_records, estimated_records, pairs, correct_scale):
    reference, estimate = as_evo(reference_records), as_evo(estimated_records)
    rotation, translation, scale = estimate.align(reference, correct_scale=correct_scale)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((reference, estimate))
    translations, rotations = [], []
    for i, j in pairs:
        error = metrics.RPE.rpe_base(reference.poses_se3[i], reference.poses_se3[j],
                                      estimate.poses_se3[i], estimate.poses_se3[j])
        translations.append(np.linalg.norm(error[:3, 3]))
        rotations.append(abs(lie_algebra.so3_log_angle(error[:3, :3], degrees=True)))
    result = {
        "alignment": "sim3" if correct_scale else "se3",
        "scale": float(scale), "rotation": rotation.tolist(), "translation": translation.tolist(),
        "ate_translation_m": {key: float(value) for key, value in ape.get_all_statistics().items()},
        "rpe_translation_m": statistics(translations), "rpe_rotation_deg": statistics(rotations),
    }
    return result, reference, estimate, ape.error, np.array(translations), np.array(rotations)


def tracking_summary(rows):
    names = {-1: "SYSTEM_NOT_READY", 0: "NO_IMAGES_YET", 1: "NOT_INITIALIZED", 2: "OK",
             3: "RECENTLY_LOST", 4: "LOST", 5: "OK_KLT"}
    states = [int(row["state"]) for row in rows]
    first = next((i for i, state in enumerate(states) if state in (2, 5)), None)
    events, current = [], None
    if first is not None:
        for i in range(first+1, len(rows)):
            if states[i] not in (2, 5) and current is None:
                current = i
            if current is not None and (states[i] in (2, 5) or i == len(rows)-1):
                last = i-1 if states[i] in (2, 5) else i
                events.append({"first_input_index": current, "last_input_index": last,
                               "start_timestamp": rows[current]["timestamp"],
                               "end_timestamp": rows[last]["timestamp"],
                               "recovered": states[i] in (2, 5)})
                current = None
    return {
        "input_frames": len(rows), "state_counts": {names[state]: states.count(state) for state in sorted(set(states))},
        "initialization_frames_before_first_ok": first if first is not None else len(rows),
        "first_ok_input_index": first,
        "tracked_frames": sum(state in (2, 5) for state in states),
        "recently_lost_frames": states.count(3), "lost_frames": states.count(4),
        "post_initialization_failure_events": events,
        "tracking_call_seconds_all_frames": statistics([float(row["tracking_seconds"]) for row in rows]),
        "tracking_call_seconds_ok_frames": statistics([float(row["tracking_seconds"]) for row in rows
                                                       if int(row["state"]) in (2, 5)]),
    }


def window_scale_diagnostic(reference, estimate, input_indices, window_seconds=5.0):
    """Sim(3)-aligned path-length ratio by fixed windows, not an observability test.

    Only adjacent original input frames contribute. Ratios contain reference
    noise and trajectory jitter and are not local Umeyama scale estimates.
    """
    if window_seconds <= 0:
        raise ValueError("Scale window duration must be positive")
    times = estimate.timestamps-estimate.timestamps[0]
    output = []
    for start in np.arange(0, times[-1]+1e-9, window_seconds):
        indices = [i for i in range(len(times)-1)
                   if start <= times[i] < start+window_seconds and times[i+1] <= start+window_seconds
                   and input_indices[i+1] == input_indices[i]+1]
        gt_length = sum(float(np.linalg.norm(reference.positions_xyz[i+1]-reference.positions_xyz[i]))
                        for i in indices)
        est_length = sum(float(np.linalg.norm(estimate.positions_xyz[i+1]-estimate.positions_xyz[i]))
                         for i in indices)
        ratio = est_length/gt_length if gt_length > 1e-6 else None
        pose_ids = [i for i, value in enumerate(times) if start <= value <= start+window_seconds]
        local_fit = None
        excursion = None
        if len(pose_ids) >= 3:
            gt_positions = reference.positions_xyz[pose_ids]
            est_positions = estimate.positions_xyz[pose_ids]
            excursion = float(np.linalg.norm(gt_positions-gt_positions[0], axis=1).max())
            try:
                _, _, correction = geometry.umeyama_alignment(est_positions.T, gt_positions.T, True)
                local_fit = {"additional_scale_on_globally_aligned_estimate": float(correction),
                             "residual_estimate_scale_percent": float((1/correction-1)*100)}
            except geometry.GeometryException as error:
                local_fit = {"unavailable": str(error)}
        output.append({"start_seconds": float(start), "end_seconds": float(min(start+window_seconds, times[-1])),
                       "adjacent_pairs": len(indices), "reference_path_length_m": gt_length,
                       "sim3_estimated_path_length_m": est_length, "path_length_ratio": ratio,
                       "relative_difference_percent": (ratio-1)*100 if ratio is not None else None,
                       "reference_max_excursion_from_window_start_m": excursion,
                       "exploratory_local_sim3_scale_fit": local_fit})
    return output
