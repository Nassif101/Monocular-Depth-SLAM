#!/usr/bin/env python3
"""Check Linux binary linkage and a headless Pangolin window, without a dataset."""
import os
from pathlib import Path
import platform
import subprocess


ROOT = Path(__file__).resolve().parents[1]


if __name__ == "__main__":
    if platform.system() != "Linux":
        raise SystemExit("Run this check inside the Ubuntu development container.")
    output = ROOT / "results/setup"
    output.mkdir(parents=True, exist_ok=True)
    orb = ROOT / "third_party/ORB_SLAM3"
    for relative in ("lib/libORB_SLAM3.so", "Examples/Monocular/mono_tum",
                     "Examples/RGB-D/rgbd_tum"):
        binary = orb / relative
        result = subprocess.run(["ldd", str(binary)], capture_output=True, text=True, check=True)
        if "not found" in result.stdout:
            raise RuntimeError(f"Unresolved library for {binary}:\n{result.stdout}")
        (output / f"{binary.name}-ldd.txt").write_text(result.stdout)
        if relative.startswith("Examples/"):
            usage = subprocess.run([str(binary)], capture_output=True, text=True, check=False)
            if usage.returncode != 1 or "Usage:" not in usage.stdout + usage.stderr:
                raise RuntimeError(f"Unexpected startup result for {binary}: {usage}")
        print(f"Binary linkage/startup: {relative}: passed", flush=True)

    prefix = ROOT / f"third_party/install/{platform.system()}-{platform.machine()}"
    source = output / "pangolin_smoke.cc"
    source.write_text(
        '#include <pangolin/pangolin.h>\n'
        'int main() { pangolin::CreateWindowAndBind("setup", 64, 64); '
        'pangolin::FinishFrame(); return 0; }\n'
    )
    executable = output / "pangolin_smoke"
    subprocess.run([
        "g++", "-std=c++14", f"-I{prefix / 'include'}", "-I/usr/include/eigen3",
        str(source), f"-L{prefix / 'lib'}", f"-Wl,-rpath,{prefix / 'lib'}",
        "-lpangolin", "-lGL", "-lGLEW", "-o", str(executable),
    ], check=True)
    subprocess.run(["xvfb-run", "-a", str(executable)], check=True,
                   env={**os.environ, "LIBGL_ALWAYS_SOFTWARE": "1"})
    print("Headless Pangolin/OpenGL window: passed")
    print("No SLAM sequence or trajectory accuracy has been evaluated.")
