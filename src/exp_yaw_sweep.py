
"""CP3 - Topic A: LiDAR-camera yaw calibration drift experiment.

Run:
python -m src.exp_yaw_sweep
"""
from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np

from starter.datasets import load_frame
from starter.projection import (
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)

CLASSES = ("Car", "Van", "Pedestrian", "Cyclist")


def points_in_box(points_cam, obj):
    """Select points inside a KITTI 3D ground-truth box."""
    h, w, l = obj.dimensions
    c = np.cos(obj.rotation_y)
    s = np.sin(obj.rotation_y)

    R = np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ])

    local = (points_cam - obj.location) @ R

    return (
        (np.abs(local[:, 0]) <= l / 2)
        & (local[:, 1] <= 0)
        & (local[:, 1] >= -h)
        & (np.abs(local[:, 2]) <= w / 2)
    )


def ratio(hits, total):
    return round(hits / total, 4) if total else float("nan")


def run_one(fr, yaw_deg):
    # Ignore invalid LiDAR points.
    points = fr["points"]
    points = points[np.isfinite(points).all(axis=1)]

    # Ground-truth camera coordinates remain unchanged.
    cam_true = velo_to_cam(points[:, :3], fr["calib"])

    # Only perturb calibration, not the original point cloud.
    calib_bad = perturb_extrinsic(
        fr["calib"],
        yaw_deg=yaw_deg,
    )

    uv, depth, mask = project_velo_to_image(
        points,
        calib_bad,
        fr["image"].shape,
    )

    uv_all = np.full((len(points), 2), np.nan)
    uv_all[mask] = uv

    total_points = 0
    total_hits = 0

    by_class = defaultdict(lambda: {"points": 0, "hits": 0})
    objects = []

    for object_id, obj in enumerate(fr["labels"]):
        if obj.type not in CLASSES:
            continue

        # Identify object points using original calibration.
        gt_mask = points_in_box(cam_true, obj)

        # Match the guide: denominator includes projected
        # object points that remain inside the image.
        selected = gt_mask & mask
        n_object = int(selected.sum())

        x1, y1, x2, y2 = obj.bbox
        object_uv = uv_all[selected]

        hits = int((
            (object_uv[:, 0] >= x1)
            & (object_uv[:, 0] <= x2)
            & (object_uv[:, 1] >= y1)
            & (object_uv[:, 1] <= y2)
        ).sum())

        distance = float(np.hypot(
            obj.location[0],
            obj.location[2],
        ))

        total_points += n_object
        total_hits += hits

        by_class[obj.type]["points"] += n_object
        by_class[obj.type]["hits"] += hits

        objects.append({
            "object_id": object_id,
            "class": obj.type,
            "yaw_deg": yaw_deg,
            "distance_m": round(distance, 2),
            "bbox_width_px": round(float(x2 - x1), 2),
            "object_points": n_object,
            "hits": hits,
            "hit_ratio": ratio(hits, n_object),
        })

    overall = {
        "n_points": len(points),
        "inside_image": int(mask.sum()),
        "object_points": total_points,
        "hits": total_hits,
        "hit_ratio": ratio(total_hits, total_points),
    }

    class_results = []
    for cls in CLASSES:
        if cls not in by_class:
            continue

        n = by_class[cls]["points"]
        h = by_class[cls]["hits"]

        class_results.append({
            "class": cls,
            "yaw_deg": yaw_deg,
            "object_points": n,
            "hits": h,
            "hit_ratio": ratio(h, n),
        })

    return overall, class_results, objects


def save_csv(path, rows):
    if not rows:
        raise ValueError(f"No data to save: {path}")

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved: {path} ({len(rows)} rows)")


def main():
    ap = argparse.ArgumentParser(
        description="LiDAR-camera yaw drift benchmark"
    )
    ap.add_argument("--data-root", default="data/kitti_mini")
    ap.add_argument(
        "--frames",
        nargs="+",
        default=["000008", "000011", "000049"],
    )
    ap.add_argument(
        "--yaw-levels",
        nargs="+",
        type=float,
        default=[0, 0.5, 1, 2, 3],
    )
    ap.add_argument(
        "--out",
        default="results/yaw_perturb_sweep.csv",
    )
    ap.add_argument(
        "--class-out",
        default="results/yaw_by_class.csv",
    )
    ap.add_argument(
        "--object-out",
        default="results/yaw_objects.csv",
    )
    args = ap.parse_args()

    overall_rows = []
    class_rows = []
    object_rows = []

    for frame in args.frames:
        fr = load_frame(args.data_root, frame)

        for yaw in args.yaw_levels:
            overall, classes, objects = run_one(fr, yaw)

            common = {
                "dataset": Path(args.data_root).name,
                "frame": frame,
            }

            row = {
                **common,
                "yaw_deg": yaw,
                **overall,
            }
            overall_rows.append(row)

            for entry in classes:
                class_rows.append({**common, **entry})

            for entry in objects:
                object_rows.append({**common, **entry})

            print(
                f"Frame={frame} yaw={yaw:.1f} "
                f"hit_ratio={overall['hit_ratio']:.4f}"
            )

    save_csv(args.out, overall_rows)
    save_csv(args.class_out, class_rows)
    save_csv(args.object_out, object_rows)


if __name__ == "__main__":
    main()
