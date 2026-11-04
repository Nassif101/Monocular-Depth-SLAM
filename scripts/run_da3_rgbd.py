#!/usr/bin/env python3
"""Replay fixed-conversion DA3 depth with ORB-SLAM3's existing RGB-D interface."""
from run_monocular import main


if __name__ == "__main__":
    main("configs/orbslam3/da3_rgbd_baseline.yaml", "results/orbslam3_da3_rgbd_fr1_xyz/run_001")
