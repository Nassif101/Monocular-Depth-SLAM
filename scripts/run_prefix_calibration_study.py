#!/usr/bin/env python3
"""Preserve three offline GT-prefix-calibrated DA3 RGB-D replays."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

import yaml

from run_monocular import sha256
from run_sensor_control_study import write_json


ROOT = Path(__file__).resolve().parents[1]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"configs/evaluations/prefix_calibration.yaml")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    prepared_path = ROOT/config["prepared_depth_directory"]/"manifest.json"
    prepared = json.loads(prepared_path.read_text())
    if prepared["status"] != "complete" or not prepared["contract"]["groundtruth_calibration"]:
        raise ValueError("Complete explicitly GT-prefix-calibrated depth is required")
    baseline_path = ROOT/config["baseline_study"]
    baseline = json.loads(baseline_path.read_text())
    if baseline["status"] != "complete":
        raise ValueError("The preserved sensor-control study must be complete")
    output = ROOT/config["output_directory"]
    output.mkdir(parents=True, exist_ok=True)
    contract = {"config_sha256": sha256(args.config), "scheduler_sha256": sha256(Path(__file__)),
                "replay_script_sha256": sha256(ROOT/"scripts/run_monocular.py"),
                "protocol_sha256": sha256(ROOT/config["replay_protocol"]),
                "prepared_manifest_sha256": sha256(prepared_path),
                "baseline_study_sha256": sha256(baseline_path),
                "repeats": config["repeats"], "frozen_depth_scale": config["frozen_depth_scale"],
                "calibration_frames": config["calibration_frames"],
                "comparison": "three new sequential replays versus nine historical control processes"}
    study_path = output/"study.json"
    if study_path.exists():
        study = json.loads(study_path.read_text())
        if study["contract"] != contract:
            raise ValueError("Preserve this study; configuration/code/depth provenance differs")
    else:
        study = {"contract": contract, "status": "scheduled", "created_utc": datetime.now(timezone.utc).isoformat(),
                 "jobs": [{"method": "da3_prefix_rgbd", "repeat": i,
                           "run_dir": str((output/"da3_prefix_rgbd"/f"repeat_{i:03d}").relative_to(ROOT)),
                           "status": "pending"} for i in range(1, config["repeats"]+1)]}
        write_json(study_path, study)
    study["status"] = "running"
    write_json(study_path, study)
    for job in study["jobs"]:
        if job["status"] != "pending":
            continue
        job.update(status="running", started_utc=datetime.now(timezone.utc).isoformat())
        write_json(study_path, study)
        print(f"Starting calibrated replay {job['repeat']}/{config['repeats']}", flush=True)
        try:
            with (output/f"repeat_{job['repeat']:03d}.log").open("w") as stream:
                result = subprocess.run([sys.executable, str(ROOT/"scripts/run_monocular.py"),
                                         "--protocol", str(ROOT/config["replay_protocol"]),
                                         "--run-dir", str(ROOT/job["run_dir"])], stdout=stream,
                                        stderr=subprocess.STDOUT, timeout=1000)
            manifest_path = ROOT/job["run_dir"]/"manifest.json"
            manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
            job.update(returncode=result.returncode,
                       status="complete" if result.returncode == 0 and manifest.get("status") == "complete" else "failed")
            if manifest_path.exists():
                job["manifest_sha256"] = sha256(manifest_path)
            if job["status"] == "complete":
                job["exported_poses"] = manifest["primary_trajectory_poses"]
        except Exception as error:
            job.update(status="failed", error=f"{type(error).__name__}: {error}")
        job["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(study_path, study)
        print(f"Finished replay {job['repeat']}: {job['status']}, poses={job.get('exported_poses')}", flush=True)
    study.update(status="complete" if all(job["status"] == "complete" for job in study["jobs"]) else "complete_with_failures",
                 finished_utc=datetime.now(timezone.utc).isoformat())
    write_json(study_path, study)
