#!/usr/bin/env python3
"""Preview baseline download sizes; download only with the explicit --download flag."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import tarfile
import tempfile
import urllib.request

import yaml


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def download(asset):
    path = ROOT / asset["path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        partial = path.with_name(path.name + ".partial")
        offset = partial.stat().st_size if partial.exists() else 0
        headers = {"Range": f"bytes={offset}-"} if offset else {}
        request = urllib.request.Request(asset["url"], headers=headers)
        with urllib.request.urlopen(request, timeout=60) as response:
            append = offset > 0 and response.status == 206
            if append and not response.headers.get("Content-Range", "").startswith(f"bytes {offset}-"):
                raise ValueError(f"Unexpected resume offset for {path}")
            received = offset if append else 0
            reported = received
            with partial.open("ab" if append else "wb") as stream:
                while block := response.read(1024 * 1024):
                    received += len(block)
                    if received > asset["size_bytes"]:
                        raise ValueError(f"Server returned more bytes than approved for {path}")
                    stream.write(block)
                    if received - reported >= 100_000_000:
                        print(f"{path.name}: {received / 1e6:.1f} MB", flush=True)
                        reported = received
        if partial.stat().st_size != asset["size_bytes"]:
            raise ValueError(f"Incomplete download: {partial}")
        digest = sha256(partial)
        if asset.get("sha256") and digest != asset["sha256"]:
            raise ValueError(f"Checksum mismatch: {partial}")
        partial.rename(path)
    else:
        if path.stat().st_size != asset["size_bytes"]:
            raise ValueError(f"Existing file has unexpected size: {path}")
        digest = sha256(path)
        if asset.get("sha256") and digest != asset["sha256"]:
            raise ValueError(f"Checksum mismatch: {path}")
    print(f"Verified {path.name}: {path.stat().st_size} bytes; sha256={digest}", flush=True)
    return {**asset, "observed_sha256": digest}


def extract_dataset(config):
    destination = ROOT / config["download"]["extract_to"]
    sequence = destination / config["sequence"]
    if sequence.exists():
        print(f"Preserving existing dataset directory: {sequence}", flush=True)
        return
    destination.mkdir(parents=True, exist_ok=True)
    archive = ROOT / config["download"]["archive"]
    with tarfile.open(archive, "r:gz") as stream:
        members = stream.getmembers()
        for member in members:
            relative = Path(member.name)
            if (relative.is_absolute() or ".." in relative.parts or
                    not relative.parts or relative.parts[0] != config["sequence"] or
                    not (member.isfile() or member.isdir())):
                raise ValueError(f"Unexpected archive entry: {member.name}")
        print(f"Extracting {sum(m.size for m in members) / 1e6:.1f} MB of files", flush=True)
        with tempfile.TemporaryDirectory(prefix=".extract-", dir=destination) as staging:
            stream.extractall(staging, members=members, filter="data")
            (Path(staging) / config["sequence"]).rename(sequence)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download", action="store_true")
    parser.add_argument("--only", choices=("all", "dataset", "checkpoint"), default="all")
    args = parser.parse_args()
    dataset = yaml.safe_load((ROOT / "configs/datasets/tum_rgbd.example.yaml").read_text())
    depth = yaml.safe_load((ROOT / "configs/da3/metric_large.yaml").read_text())
    assets = []
    if args.only in ("all", "dataset"):
        source = dataset["download"]
        assets.append({"url": source["url"], "path": source["archive"],
                       "size_bytes": source["size_bytes"], "sha256": source["sha256"]})
    if args.only in ("all", "checkpoint"):
        for name, metadata in depth["files"].items():
            assets.append({"url": f"https://huggingface.co/{depth['checkpoint']}/resolve/"
                           f"{depth['checkpoint_revision']}/{name}",
                           "path": f"{depth['local_directory']}/{name}", **metadata})
    for asset in assets:
        print(f"{asset['path']}: {asset['size_bytes']} bytes ({asset['size_bytes']/1e6:.3f} MB)",
              flush=True)
    total = sum(asset["size_bytes"] for asset in assets)
    print(f"Total download: {total} bytes ({total/1e9:.3f} GB), plus extracted dataset storage.",
          flush=True)
    if args.download:
        with ThreadPoolExecutor(max_workers=2) as pool:
            records = list(pool.map(download, assets))
        if args.only in ("all", "dataset"):
            extract_dataset(dataset)
        report = ROOT / "results/downloads" / f"{args.only}.json"
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps({"timestamp_utc": datetime.now(timezone.utc).isoformat(),
                                     "assets": records}, indent=2) + "\n")
