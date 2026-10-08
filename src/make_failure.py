"""CP4: Visualize pedestrian projection failure at yaw 2 degrees."""

from pathlib import Path

import cv2
import numpy as np

from starter.datasets import load_frame
from starter.projection import (
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)
from src.exp_yaw_sweep import points_in_box, run_one

FRAME = "000011"
YAW_BAD = 2.0

# Load original KITTI frame.
fr = load_frame("data/kitti_mini", FRAME)

# Get per-object benchmark results.
base = {r["object_id"]: r for r in run_one(fr, 0.0)[2]}
bad = {r["object_id"]: r for r in run_one(fr, YAW_BAD)[2]}

# Find pedestrian most affected by yaw drift.
candidates = [
    i
    for i, r in base.items()
    if r["class"] == "Pedestrian"
    and r["object_points"] >= 10
    and bad[i]["object_points"] >= 10
    and r["hit_ratio"] >= 0.9
]

if not candidates:
    raise RuntimeError("No suitable pedestrian found")

target_id = max(
    candidates,
    key=lambda i: (base[i]["hit_ratio"] - bad[i]["hit_ratio"]),
)

obj = fr["labels"][target_id]

# Filter invalid LiDAR points.
points = fr["points"]
points = points[np.isfinite(points).all(axis=1)]

# Identify points belonging to the selected pedestrian
# using the original calibration.
cam_true = velo_to_cam(
    points[:, :3],
    fr["calib"],
)
object_mask = points_in_box(cam_true, obj)

H, W = fr["image"].shape[:2]
x1, y1, x2, y2 = [int(round(x)) for x in obj.bbox]

# Use identical crop boundaries for comparison.
left = max(0, x1 - 90)
right = min(W, x2 + 90)
top = max(0, y1 - 60)
bottom = min(H, y2 + 60)


def make_panel(yaw):
    """Generate one visualization panel."""
    calib = perturb_extrinsic(
        fr["calib"],
        yaw_deg=yaw,
    )

    uv, _, mask = project_velo_to_image(
        points,
        calib,
        fr["image"].shape,
    )

    # Only draw projected points belonging to this object.
    object_uv = uv[object_mask[mask]]

    vis = fr["image"].copy()

    # Draw LiDAR points in red.
    for u, v in object_uv:
        cv2.circle(
            vis,
            (int(round(u)), int(round(v))),
            2,
            (0, 0, 255),
            -1,
        )

    # Draw ground-truth 2D bounding box in green.
    cv2.rectangle(
        vis,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2,
    )

    crop = vis[top:bottom, left:right]

    score = base[target_id] if yaw == 0 else bad[target_id]

    # Add a title to the panel.
    header = np.full(
        (65, crop.shape[1], 3),
        245,
        dtype=np.uint8,
    )

    cv2.putText(
        header,
        f"Yaw: {yaw:.1f} deg",
        (8, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        1,
    )

    cv2.putText(
        header,
        f"Hit ratio: {score['hit_ratio']:.1%}",
        (8, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        1,
    )

    return np.vstack([header, crop])


# Compare correct and incorrect calibration.
comparison = np.hstack(
    [
        make_panel(0.0),
        make_panel(YAW_BAD),
    ]
)

# Enlarge visualization.
comparison = cv2.resize(
    comparison,
    None,
    fx=2,
    fy=2,
    interpolation=cv2.INTER_NEAREST,
)

# Save failure evidence.
out = Path("results/figures/fail_01_yaw_2deg_pedestrian.png")
out.parent.mkdir(parents=True, exist_ok=True)

if not cv2.imwrite(str(out), comparison):
    raise RuntimeError("Failed to save failure image")

# Numerical analysis.
drop = 100 * (base[target_id]["hit_ratio"] - bad[target_id]["hit_ratio"])

print(f"Frame: {FRAME}")
print(f"Object ID: {target_id}")
print(f"Distance: {base[target_id]['distance_m']} m")
print(f"Baseline: {base[target_id]['hit_ratio']:.2%}")
print(f"Yaw 2 deg: {bad[target_id]['hit_ratio']:.2%}")
print(f"Drop: {drop:.2f} percentage points")
print(f"Saved: {out}")
