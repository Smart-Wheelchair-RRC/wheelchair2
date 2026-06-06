import numpy as np
import open3d as o3d
from matplotlib.pyplot import imsave
import yaml, os
from scipy.ndimage import (minimum_filter, binary_closing, binary_opening,
                           generic_filter, distance_transform_edt)

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG — tune these first before anything else
# ─────────────────────────────────────────────────────────────────────────────
MAP_NAME   = "lidarnewmap2d22"
PCD_PATH   = "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/class/22may/lidarnewmap1.pcd"
OUT_PATH   = "/home/container_user/wheelchair2/src/wheelchair2_navigation/maps/class/22may"

RES               = 0.05   # m/cell — map resolution
FLOOR_CELL_SIZE   = 0.4   # m — coarse cell size for floor estimation (larger = smoother)
FLOOR_PERCENTILE  = 8      # % — use 8th-percentile Z as floor (robust against noise/legs)
FLOOR_TOLERANCE   = 0.12   # m — points this close above floor = free space
OBS_MIN_H         = 0.15   # m — min height above floor to count as obstacle
OBS_MAX_H         = 1.8    # m — max height above floor (ignore ceiling returns)

# Statistical outlier removal — removes floating noise points
SOR_NEIGHBORS     = 20     # neighbours to check
SOR_STD_RATIO     = 2.0    # points > this many std-devs from mean distance = outlier

# ─────────────────────────────────────────────────────────────────────────────
# 1. LOAD + CLEAN
# ─────────────────────────────────────────────────────────────────────────────
print("Loading PCD …")
pcd_o3d = o3d.io.read_point_cloud(PCD_PATH)

print(f"  Raw points : {len(pcd_o3d.points):,}")
pcd_o3d, _ = pcd_o3d.remove_statistical_outlier(
    nb_neighbors=SOR_NEIGHBORS, std_ratio=SOR_STD_RATIO)
print(f"  After SOR  : {len(pcd_o3d.points):,}")

pts = np.asarray(pcd_o3d.points)   # (N, 3)

x_min, x_max = pts[:, 0].min(), pts[:, 0].max()
y_min, y_max = pts[:, 1].min(), pts[:, 1].max()

W = int((x_max - x_min) / RES) + 1
H = int((y_max - y_min) / RES) + 1
print(f"  Fine grid  : {W} × {H}")

# ─────────────────────────────────────────────────────────────────────────────
# 2. ROBUST FLOOR ESTIMATION  (percentile Z per coarse cell)
# ─────────────────────────────────────────────────────────────────────────────
FC  = FLOOR_CELL_SIZE
FW  = int((x_max - x_min) / FC) + 1
FH  = int((y_max - y_min) / FC) + 1

# Bucket every point into its coarse cell
fx_idx = np.clip(((pts[:, 0] - x_min) / FC).astype(int), 0, FW - 1)
fy_idx = np.clip(((pts[:, 1] - y_min) / FC).astype(int), 0, FH - 1)
cell_id = fy_idx * FW + fx_idx           # 1-D cell index

# Collect Z values per cell then take percentile (vectorised with sort)
order      = np.argsort(cell_id)
cell_id_s  = cell_id[order]
z_sorted   = pts[order, 2]

floor_grid = np.full(FH * FW, np.nan)
unique_cells, starts, counts = np.unique(
    cell_id_s, return_index=True, return_counts=True)

for uc, st, cnt in zip(unique_cells, starts, counts):
    z_in_cell = z_sorted[st : st + cnt]
    floor_grid[uc] = np.percentile(z_in_cell, FLOOR_PERCENTILE)

floor_grid = floor_grid.reshape(FH, FW)

# Fill NaN cells via nearest-neighbour propagation
nan_mask = np.isnan(floor_grid)
if nan_mask.any():
    _, idx = distance_transform_edt(nan_mask, return_indices=True)
    floor_grid = floor_grid[idx[0], idx[1]]

# Smooth with a min-filter so bumps in the floor estimation don't spike
k = max(1, int(0.6 / FC))              # ~0.6 m smoothing radius
floor_grid = minimum_filter(floor_grid, size=2 * k + 1)

# ─────────────────────────────────────────────────────────────────────────────
# 3. CLASSIFY EVERY POINT
# ─────────────────────────────────────────────────────────────────────────────
# Fine-grid indices for all points
xi = np.clip(((pts[:, 0] - x_min) / RES).astype(int), 0, W - 1)
yi = np.clip(((pts[:, 1] - y_min) / RES).astype(int), 0, H - 1)

# Map each fine cell → coarse floor cell
fx = np.clip(((pts[:, 0] - x_min) / FC).astype(int), 0, FW - 1)
fy = np.clip(((pts[:, 1] - y_min) / FC).astype(int), 0, FH - 1)
floor_z = floor_grid[fy, fx]

dz = pts[:, 2] - floor_z              # height above estimated floor

is_floor   = dz <= FLOOR_TOLERANCE
is_obs     = (dz > OBS_MIN_H) & (dz <= OBS_MAX_H)

# Image row = flip Y
yi_img = H - 1 - yi

# ── accumulate votes per cell ─────────────────────────────────────────────
obs_count   = np.zeros((H, W), dtype=np.int32)
floor_count = np.zeros((H, W), dtype=np.int32)

np.add.at(floor_count, (yi_img[is_floor], xi[is_floor]), 1)
np.add.at(obs_count,   (yi_img[is_obs],   xi[is_obs]),   1)

# A cell is free if floor votes > 0 AND obstacle votes are sparse
# Use a vote ratio to handle mixed cells (glass walls, thin objects)
total = floor_count + obs_count
with np.errstate(divide='ignore', invalid='ignore'):
    obs_ratio = np.where(total > 0, obs_count / total, 0.0)

is_free_cell = (floor_count > 0) & (obs_ratio < 0.3)
is_obs_cell  = obs_count >= 2               # require ≥2 obstacle points per cell

# ─────────────────────────────────────────────────────────────────────────────
# 4. MORPHOLOGICAL CLEANUP
# ─────────────────────────────────────────────────────────────────────────────
# Remove isolated obstacle pixels (sensor blips)
is_obs_cell  = binary_opening(is_obs_cell,  iterations=1)
# Seal thin wall gaps
is_obs_cell  = binary_closing(is_obs_cell,  iterations=1)
# Expand free space to fill shadow zones near walls
is_free_cell = binary_closing(is_free_cell, iterations=3)

# Obstacles always override free (do this AFTER morphology)
is_free_cell[is_obs_cell] = False

# ─────────────────────────────────────────────────────────────────────────────
# 5. INPAINT UNKNOWN ISLANDS surrounded by free space
#    (these are the "black holes" in your map)
# ─────────────────────────────────────────────────────────────────────────────
unknown_mask = ~is_free_cell & ~is_obs_cell

# Any unknown cell that has ≥ 4 free neighbours gets promoted to free
def has_free_neighbour(patch):
    centre = patch[len(patch) // 2]
    if centre == 0:          # not unknown
        return 0
    return 1 if patch.sum() >= 4 else 0   # patch values are 1 for free

# Iterative flood-fill promotion (run a few times to expand inward)
for _ in range(6):
    free_u8 = is_free_cell.astype(np.uint8)
    promoted = generic_filter(free_u8, has_free_neighbour,
                              size=3,
                              extra_arguments=(),
                              mode='constant', cval=0) & unknown_mask
    is_free_cell |= promoted.astype(bool)
    unknown_mask  = ~is_free_cell & ~is_obs_cell

# ─────────────────────────────────────────────────────────────────────────────
# 6. RENDER  (ROS map_server convention)
#    254 = free (white), 205 = unknown (grey), 0 = occupied (black)
# ─────────────────────────────────────────────────────────────────────────────
img = np.full((H, W), 205, dtype=np.uint8)
img[is_free_cell] = 254
img[is_obs_cell]  = 0

# ─────────────────────────────────────────────────────────────────────────────
# 7. SAVE
# ─────────────────────────────────────────────────────────────────────────────
png_path  = os.path.join(OUT_PATH, f"{MAP_NAME}.png")
yaml_path = os.path.join(OUT_PATH, f"{MAP_NAME}.yaml")

imsave(png_path, img, cmap="gray", vmin=0, vmax=255)

map_dict = {
    "image":           f"{MAP_NAME}.png",
    "resolution":      RES,
    "origin":          [float(x_min), float(y_min), 0.0],
    "occupied_thresh": 0.65,
    "free_thresh":     0.25,
    "negate":          0,
}
with open(yaml_path, "w") as f:
    yaml.dump(map_dict, f, default_flow_style=None)

print(f"\nDone → {png_path}")
print(f"       {yaml_path}")

# ── quick diagnostic printout ─────────────────────────────────────────────
total_cells = H * W
print(f"\nCell breakdown:")
print(f"  Free     : {is_free_cell.sum():>8,}  ({100*is_free_cell.sum()/total_cells:.1f}%)")
print(f"  Obstacle : {is_obs_cell.sum():>8,}  ({100*is_obs_cell.sum()/total_cells:.1f}%)")
print(f"  Unknown  : {unknown_mask.sum():>8,}  ({100*unknown_mask.sum()/total_cells:.1f}%)")