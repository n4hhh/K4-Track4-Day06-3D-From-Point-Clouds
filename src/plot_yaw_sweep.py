"""Plot yaw drift benchmark results."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OUT = Path("results/figures")
OUT.mkdir(parents=True, exist_ok=True)

# Figure 1: hit ratio by frame.
df = pd.read_csv(
    "results/yaw_perturb_sweep.csv",
    dtype={"frame": str},
)

fig, ax = plt.subplots(figsize=(8, 5))

for frame, group in df.groupby("frame"):
    group = group.sort_values("yaw_deg")
    ax.plot(
        group["yaw_deg"],
        group["hit_ratio"] * 100,
        marker="o",
        label=f"Frame {frame}",
    )

ax.set_xlabel("Yaw calibration error (degrees)")
ax.set_ylabel("Object point hit ratio (%)")
ax.set_title("Calibration drift sensitivity by frame")
ax.set_ylim(0, 105)
ax.grid(alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "yaw_sweep.png", dpi=160)
plt.close(fig)

# Figure 2: hit ratio aggregated by class.
df_class = pd.read_csv("results/yaw_by_class.csv")

fig, ax = plt.subplots(figsize=(8, 5))

for cls, group in df_class.groupby("class"):
    agg = (
        group.groupby("yaw_deg")[["hits", "object_points"]]
        .sum()
        .reset_index()
        .sort_values("yaw_deg")
    )

    valid = agg["object_points"] > 0

    ax.plot(
        agg.loc[valid, "yaw_deg"],
        (agg.loc[valid, "hits"] / agg.loc[valid, "object_points"]) * 100,
        marker="o",
        label=cls,
    )

ax.set_xlabel("Yaw calibration error (degrees)")
ax.set_ylabel("Object point hit ratio (%)")
ax.set_title("Calibration drift sensitivity by class")
ax.set_ylim(0, 105)
ax.grid(alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "yaw_by_class.png", dpi=160)
plt.close(fig)

print("Saved results/figures/yaw_sweep.png")
print("Saved results/figures/yaw_by_class.png")
