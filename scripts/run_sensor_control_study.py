#!/usr/bin/env python3
"""Run nine preserved, sequential, balanced-order sensor-control replays."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import random
import subprocess
import sys

import yaml

from monocular_depth_slam.datasets.tum import load_config
from monocular_depth_slam.slam.inputs import selected_frames
from run_monocular import sha256


ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(value, indent=2)+"\n")
    temporary.replace(path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"configs/evaluations/sensor_depth_control.yaml")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if config["repeats_per_method"] != 3 or len(config["methods"]) != 3:
        raise ValueError("This balanced order implements exactly three methods and three repetitions")
    sequence = load_config(ROOT/config["dataset_config"], project_root=ROOT)
    frames = selected_frames(sequence, config)
    if len(frames) != config["expected_input_frames"]:
        raise ValueError("Unexpected sensor-matched RGB frame count")
    output = ROOT/config["output_directory"]
    output.mkdir(parents=True, exist_ok=True)
    path = output/"study.json"
    contract = {"config_sha256": sha256(args.config), "script_sha256": sha256(Path(__file__)),
                "replay_script_sha256": sha256(ROOT/"scripts/run_monocular.py"),
                "dataset_config_sha256": sha256(ROOT/config["dataset_config"]),
                "method_protocol_sha256": {name: sha256(ROOT/file) for name,file in config["methods"].items()},
                "input_frames": len(frames), "seed": config["schedule_seed"],
                "schedule": "Seeded Latin square: every method once per block and once per within-block position",
                "replicate_unit": "fresh process with fixed seed; one independent scene"}
    if path.exists():
        study = json.loads(path.read_text())
        if study["contract"] != contract:
            raise ValueError("Study provenance differs; preserve this study and choose another output directory")
    else:
        labels = list(config["methods"])
        generator = random.Random(config["schedule_seed"])
        generator.shuffle(labels)
        blocks = list(range(3))
        generator.shuffle(blocks)
        jobs = []
        repeats = dict.fromkeys(labels, 0)
        for block, row in enumerate(blocks):
            for position in range(3):
                method = labels[(row+position) % 3]
                repeats[method] += 1
                directory = output/method/f"repeat_{repeats[method]:03d}"
                jobs.append({"order": len(jobs)+1, "block": block+1, "position": position+1,
                             "method": method, "repeat": repeats[method],
                             "protocol": config["methods"][method],
                             "run_dir": str(directory.relative_to(ROOT)), "status": "pending"})
        study = {"contract": contract, "created_utc": datetime.now(timezone.utc).isoformat(),
                 "status": "scheduled", "jobs": jobs}
        write_json(path, study)
    print("Frozen schedule: "+", ".join(f"{job['method']}[{job['repeat']}]" for job in study["jobs"]), flush=True)
    study["status"] = "running"
    write_json(path, study)
    for job in study["jobs"]:
        if job["status"] != "pending":
            continue
        job.update(status="running", started_utc=datetime.now(timezone.utc).isoformat())
        write_json(path, study)
        print(f"Starting {job['order']}/9: {job['method']} repeat {job['repeat']}", flush=True)
        try:
            with (output/f"job_{job['order']:02d}.log").open("w") as stream:
                result = subprocess.run([sys.executable, str(ROOT/"scripts/run_monocular.py"),
                                         "--protocol", str(ROOT/job["protocol"]),
                                         "--run-dir", str(ROOT/job["run_dir"])],
                                        stdout=stream, stderr=subprocess.STDOUT, timeout=1000)
            job["returncode"] = result.returncode
            run_manifest = ROOT/job["run_dir"]/"manifest.json"
            manifest = json.loads(run_manifest.read_text()) if run_manifest.exists() else {}
            job["status"] = "complete" if result.returncode == 0 and manifest.get("status") == "complete" else "failed"
            if run_manifest.exists():
                job["manifest_sha256"] = sha256(run_manifest)
            if job["status"] == "complete":
                job.update(exported_poses=manifest["primary_trajectory_poses"],
                           maps=manifest["atlas_map_count_at_export"], keyframes=manifest["largest_map_keyframes"])
        except Exception as error:
            job.update(status="failed", error=f"{type(error).__name__}: {error}")
        job["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(path, study)
        print(f"Finished {job['order']}/9: {job['status']}, exported={job.get('exported_poses')}", flush=True)
    study["status"] = "complete" if all(job["status"] == "complete" for job in study["jobs"]) else "complete_with_failures"
    study["finished_utc"] = datetime.now(timezone.utc).isoformat()
    write_json(path, study)
    print(study["status"], flush=True)
