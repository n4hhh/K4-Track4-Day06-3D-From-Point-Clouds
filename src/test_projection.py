
"""CP2: numerical projection self-test."""
import numpy as np

from starter.datasets import load_frame
from starter.projection import cam_to_image, velo_to_cam

fr = load_frame("data/synthetic", "000000")
calib = fr["calib"]
shape = fr["image"].shape

pts = np.array([
    [10.0, 0.0, 0.0],
    [np.nan, 0.0, 0.0],
    [-10.0, 0.0, 0.0],
    [10.0, 50.0, 0.0],
])

cam = velo_to_cam(pts, calib)
uv, depth, mask = cam_to_image(cam, calib.P2, shape)

print("Camera coordinates:\n", np.round(cam, 2))
print("UV:", np.round(uv, 1))
print("Depth:", np.round(depth, 2))
print("Mask:", mask)

assert cam.shape == (4, 3)
assert abs(cam[0, 2] - 9.73) < 0.01
assert mask.tolist() == [True, False, False, False]
assert uv.shape == (1, 2)
assert depth.shape == (1,)
assert np.allclose(uv[0], [614, 175], atol=1)

print("CP2 self-test passed")
