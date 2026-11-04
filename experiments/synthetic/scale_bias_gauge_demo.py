#!/usr/bin/env python3
"""Exact monocular scale / learned-depth bias gauge demonstration.

Uses only the Python standard library. No real image data, DA3, or SLAM.
"""
from math import log, hypot, isclose


def project(point, center):
    x, y, z = (p - c for p, c in zip(point, center))
    return (x / z, y / z)


def log_depth_residual(point, center, beta, log_depth_observation):
    z = point[2] - center[2]
    return log(z) + beta - log_depth_observation


def ray_distance(point, center, image_uv):
    # Cross-product norm / ray norm; equivalent to Pluecker point-to-ray distance.
    dx, dy, dz = image_uv[0], image_uv[1], 1.0
    x, y, z = (p - c for p, c in zip(point, center))
    cross = (dy * z - dz * y, dz * x - dx * z, dx * y - dy * x)
    return hypot(hypot(cross[0], cross[1]), cross[2]) / hypot(hypot(dx, dy), dz)


def main():
    point = (1.0, 0.2, 4.0)
    center = (0.3, 0.0, 0.0)
    beta = log(1.15)
    uv = project(point, center)
    observed = log(point[2] - center[2]) + beta
    base_residual = log_depth_residual(point, center, beta, observed)
    base_ray = ray_distance(point, center, uv)
    for scale in (0.5, 1.0, 2.0, 5.0):
        p = tuple(scale * v for v in point)
        c = tuple(scale * v for v in center)
        b = beta - log(scale)
        assert all(isclose(a, d, abs_tol=1e-12) for a, d in zip(project(p, c), uv))
        assert isclose(log_depth_residual(p, c, b, observed), base_residual, abs_tol=1e-12)
        assert isclose(ray_distance(p, c, uv), scale * base_ray, abs_tol=1e-12)
        print(f"scale={scale:.2f}, reprojection unchanged, log-depth residual unchanged")
    print("PASS: geometry scale and free depth bias have an exact gauge.")


if __name__ == "__main__":
    main()
