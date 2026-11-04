#!/usr/bin/env python3
"""Report setup status; does not claim dependencies are installed."""
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[1]
for name in ("cmake", "git", "python3"):
    print(f"{name}: {shutil.which(name) or 'MISSING'}")
for name in ("datasets", "predictions", "results", "third_party"):
    print(f"{name}: {'present' if (root / name).is_dir() else 'MISSING'}")
print("ORB-SLAM3 and DA3 are external dependencies and are not installed by this scaffold.")
