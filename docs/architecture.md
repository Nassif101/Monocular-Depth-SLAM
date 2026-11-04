# Architecture

```text
TUM RGB + timestamps + calibration
             |
       +-----+------+
       |            |
 ORB-SLAM3 mono   DA3 inference
       |            |
 trajectory      cached depth + metadata
       |            |
       +------+- ----+
              |
    depth-assisted RGB-D experiment
              |
    evaluation / reproducible manifests
```

Initial fusion is an exploratory RGB-D-compatible adapter. Later work may add explicit depth factors, confidence, and bias states to local bundle adjustment. All outputs must distinguish predicted depth from sensor ground truth.
