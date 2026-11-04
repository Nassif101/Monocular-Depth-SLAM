"""Minimal experiment metadata validation."""
from pathlib import Path
import json


REQUIRED = ("dataset", "sequence", "slam_revision", "depth_revision", "alignment")


def validate_manifest(manifest: dict) -> None:
    missing = [key for key in REQUIRED if not manifest.get(key)]
    if missing:
        raise ValueError(f"Missing required run metadata: {', '.join(missing)}")
    if manifest["alignment"] not in ("se3", "sim3"):
        raise ValueError("alignment must be se3 or sim3")


def save_manifest(path: str | Path, manifest: dict) -> None:
    validate_manifest(manifest)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(manifest, indent=2) + "\n")
