#!/usr/bin/env python3
"""Check DA3 imports and device support, optionally with a random-weight forward pass."""
import argparse
import json

import torch
from depth_anything_3.api import DepthAnything3


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=("auto", "cpu", "mps", "cuda"), default="auto")
    parser.add_argument("--forward", action="store_true",
                        help="Run DA3-Small with random weights; downloads no checkpoint")
    args = parser.parse_args()
    device = args.device
    if device == "auto":
        device = ("cuda" if torch.cuda.is_available() else
                  "mps" if torch.backends.mps.is_available() else "cpu")
    tensor = torch.ones((2, 2), device=device)
    assert (tensor @ tensor).cpu().tolist() == [[2.0, 2.0], [2.0, 2.0]]
    report = {
        "torch_version": torch.__version__,
        "cuda_runtime": torch.version.cuda,
        "mps_available": torch.backends.mps.is_available(),
        "cuda_available": torch.cuda.is_available(),
        "selected_device": device,
        "da3_api_import": True,
        "tensor_operation": "passed",
        "checkpoint_loaded": False,
        "baseline_validated": False,
    }
    if args.forward:
        model = DepthAnything3("da3-small").to(device)
        image = torch.rand((1, 1, 3, 56, 56), device=device)
        with torch.inference_mode():
            prediction = model(image, export_feat_layers=[])
        depth = prediction["depth"]
        if not torch.isfinite(depth).all().item():
            raise RuntimeError("DA3 returned non-finite depth during the smoke check")
        report["random_weight_forward"] = {"status": "passed", "depth_shape": list(depth.shape)}
    print(json.dumps(report, indent=2))
