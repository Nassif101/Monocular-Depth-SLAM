#!/usr/bin/env python3
"""Report the active environment and distinguish sources from built dependencies."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def environment_report():
    root = Path(__file__).resolve().parents[1]
    packages = {}
    for name in ("monocular-depth-slam", "numpy", "PyYAML", "pytest", "ruff",
                 "evo", "scipy", "matplotlib", "torch", "depth-anything-3"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None

    sources = {}
    for name in ("ORB_SLAM3", "Pangolin", "Depth-Anything-3"):
        directory = root / "third_party" / name
        revision = None
        if (directory / ".git").exists() and shutil.which("git"):
            result = subprocess.run(
                ["git", "-C", str(directory), "rev-parse", "HEAD"],
                capture_output=True, text=True, check=False,
            )
            if result.returncode == 0:
                revision = result.stdout.strip()
        sources[name] = revision

    orb = root / "third_party" / "ORB_SLAM3"
    return {
        "platform": f"{platform.system()} {platform.release()} {platform.machine()}",
        "python": {"version": platform.python_version(), "executable": sys.executable},
        "tools": {name: shutil.which(name) for name in
                  ("uv", "cmake", "ninja", "pkg-config", "git", "docker")},
        "packages": packages,
        "source_revisions": sources,
        "orb_slam3": {
            "library_present": (orb / "lib/libORB_SLAM3.so").is_file(),
            "monocular_executable_present": (orb / "Examples/Monocular/mono_tum").is_file(),
            "rgbd_executable_present": (orb / "Examples/RGB-D/rgbd_tum").is_file(),
            "vocabulary_present": (orb / "Vocabulary/ORBvoc.txt").is_file(),
        },
        "baseline_validated": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="Emit machine-readable metadata")
    args = parser.parse_args()
    report = environment_report()
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"Platform: {report['platform']}")
        print(f"Python: {report['python']['version']} ({sys.executable})")
        for section in ("tools", "packages", "source_revisions", "orb_slam3"):
            print(f"\n{section}:")
            for name, value in report[section].items():
                print(f"  {name}: {value if value is not None else 'MISSING'}")
        print("\nDependency checks do not validate a SLAM or DA3 baseline.")
