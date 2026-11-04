#!/usr/bin/env python3
"""Fetch the exact official source revisions in configs/dependencies.json."""
import argparse
import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def fetch(name, dependency):
    directory = ROOT / "third_party" / name
    if not directory.exists():
        subprocess.run(["git", "init", str(directory)], check=True)
        subprocess.run(["git", "-C", str(directory), "remote", "add", "origin",
                        dependency["url"]], check=True)
        subprocess.run(["git", "-C", str(directory), "fetch", "--depth", "1", "origin",
                        dependency["revision"]], check=True)
        subprocess.run(["git", "-C", str(directory), "checkout", "--detach", "FETCH_HEAD"],
                       check=True)
    revision = subprocess.check_output(
        ["git", "-C", str(directory), "rev-parse", "HEAD"], text=True,
    ).strip()
    if revision != dependency["revision"]:
        raise SystemExit(f"{name}: expected {dependency['revision']}, found {revision}. "
                         "Existing checkouts are preserved; resolve the mismatch manually.")
    print(f"{name}: {revision}", flush=True)


if __name__ == "__main__":
    dependencies = json.loads((ROOT / "configs/dependencies.json").read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("names", nargs="*", help="Dependency names (default: all)")
    args = parser.parse_args()
    for name in args.names or dependencies:
        if name not in dependencies:
            parser.error(f"Unknown dependency: {name}")
        fetch(name, dependencies[name])
