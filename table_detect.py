"""
Table detection from a single RGB-D frame.

Pipeline:
  1. Load color + depth images and the camera intrinsics.
  2. Back-project every valid depth pixel into a 3D point (camera frame).
  3. Find the table as a large, roughly horizontal plane (RANSAC).
  4. Report the table's height/normal/center and save a picture with the table in red.
  5. Find a clear circle on the table where a round bowl could be put down.

Where can the bowl go? (the "free space" finder, in plain English)
  Imagine standing straight above the table and drawing a 5 mm chequerboard on it.
  Every square gets one of three labels:
    FREE      - we can see bare tabletop there (points within 2 cm of the plane).
    OCCUPIED  - we can see something sitting there: points 1 cm to 40 cm above
                the plane, on the camera's side of it. Every depth point counts,
                not just the ones that landed on the table.
    UNKNOWN   - nothing was measured: past the table edge, out of frame, or in
                the shadow behind an object. We treat UNKNOWN as occupied, so the
                bowl is never placed over a blind spot or off the edge.
  Then, for every FREE square, we work out how far it is to the nearest blocked
  square (scipy's distance transform does all of them at once). Blocked squares
  come in two kinds, with different safety gaps:
    EDGE      - anything outside the table's outline (past the edge). The bowl
                must stay R + edge_margin away (default 8 cm gap), so it is never
                set down near the edge where it could tip or fall.
    OBSTACLE  - objects on the table, and the blind spots/holes inside the table's
                outline. The bowl must stay R + margin away (default 2 cm gap).
  Of all the squares where it fits we keep the one closest to the
  middle of the camera's view (where the optical axis hits the table), because
  that is the spot the arm is already looking at. If no square is far enough
  from everything, there is nowhere to put the bowl and we say why.

Run at home on Open3D's sample frame:
    python table_detect.py
Run on your own saved frame (e.g. from the robot's RealSense):
    python table_detect.py --color color.png --depth depth.png --fx 615 --fy 615 --cx 320 --cy 240 --depth-scale 1000
"""

import argparse

import numpy as np
from scipy import ndimage

# Placement defaults - change them here and both the CLI and the ROS node pick them up.
BOWL_RADIUS = 0.055   # m, radius of the bowl (5.5 cm)
MARGIN = 0.02         # m, gap kept between the bowl and objects on the table
EDGE_MARGIN = 0.08    # m, gap kept between the bowl and the table edge
CELL_SIZE = 0.005     # m, top-down grid resolution (5 mm squares)

# Floor rejection - a plane that fails either check is never picked as the table.
# At the scan pose the tabletop is ~0.49 m from the camera (distance along the plane
# normal, measured on robot_depth.png); the floor is a whole table height further.
TABLE_DIST = 0.49                     # m, camera-to-table distance at the scan pose
MAX_PLANE_DIST = TABLE_DIST + 0.30    # m, planes farther than this are skipped
MIN_TABLE_HEIGHT = 0.0   # m, lowest allowed table height in the arm base frame; needs the
                         # camera pose (TF or place_bowl.py's FK). The table measured 0.135 m
                         # above the arm base (2026-10-04); the floor is far below 0. None = off.
# These are for the wrist camera at the scan pose. The scene camera has its own values
# in table_detector_node.py (CAMERAS).

# Open3D is optional: nicer/faster plane fitting, image loading and a 3D viewer.
# Without it (e.g. hard to install on the Jetson) we fall back to NumPy.
try:
    import open3d as o3d
    o3d.utility.random.seed(0)  # make RANSAC give the same answer every run
except ImportError:
    o3d = None


def read_image(path):
    if o3d is not None:
        return np.asarray(o3d.io.read_image(path))
    import cv2  # comes with ROS installs
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    return img[..., ::-1] if img.ndim == 3 else img  # BGR -> RGB


def write_image(path, img):
    if o3d is not None:
        o3d.io.write_image(path, o3d.geometry.Image(np.ascontiguousarray(img)))
        return
    import cv2
    cv2.imwrite(path, img[..., ::-1] if img.ndim == 3 else img)


def ransac_plane_numpy(points, dist_thresh, iterations, rng=np.random.default_rng(0)):
    """Same idea as Open3D's segment_plane, in plain NumPy.

    Repeatedly pick 3 random points, build the plane through them, count how many
    points lie within dist_thresh of it, and keep the plane with the most.
    """
    n = len(points)
    best_count, best_model = -1, None
    tri = rng.integers(0, n, size=(iterations, 3))
    p0, p1, p2 = points[tri[:, 0]], points[tri[:, 1]], points[tri[:, 2]]
    normals = np.cross(p1 - p0, p2 - p0)
    lens = np.linalg.norm(normals, axis=1)
    ok = lens > 1e-9
    normals, p0 = normals[ok] / lens[ok, None], p0[ok]
    ds = -np.sum(normals * p0, axis=1)
    sample = points if n <= 20000 else points[rng.choice(n, 20000, replace=False)]
    for nrm, d in zip(normals, ds):  # score on a subsample for speed
        count = np.count_nonzero(np.abs(sample @ nrm + d) < dist_thresh)
        if count > best_count:
            best_count, best_model = count, np.append(nrm, d)
    inliers = np.flatnonzero(np.abs(points @ best_model[:3] + best_model[3]) < dist_thresh)
    # The winning plane went exactly through 3 random points, so it inherits their
    # noise. Refit it through ALL of its inliers (least squares via SVD) - that is
    # worth ~1 cm of height accuracy, which matters when putting a bowl down.
    inlier_pts = points[inliers]
    centroid = inlier_pts.mean(axis=0)
    normal = np.linalg.svd(inlier_pts - centroid, full_matrices=False)[2][-1]
    model = np.append(normal, -normal @ centroid)
    inliers = np.flatnonzero(np.abs(points @ model[:3] + model[3]) < dist_thresh)
    return model, inliers


# ---------------------------------------------------------------------------
# Step 1: load a frame and back-project it to 3D
# ---------------------------------------------------------------------------

def load_sample_frame():
    """Open3D's TUM sample: an office desk with a floor and a wall behind it."""
    if o3d is None:
        raise SystemExit("The built-in sample needs Open3D: pip install open3d")
    data = o3d.data.SampleTUMRGBDImage()
    color = read_image(data.color_path)
    depth_raw = read_image(data.depth_path)
    # TUM intrinsics (Kinect-style camera), depth stored as 1/5000 m per unit.
    intrinsics = dict(fx=525.0, fy=525.0, cx=319.5, cy=239.5)
    depth_m = depth_raw.astype(np.float32) / 5000.0
    return color, depth_m, intrinsics


def load_frame(color_path, depth_path, depth_scale, intrinsics):
    """Your own frame. RealSense depth PNGs are usually millimeters -> depth_scale=1000."""
    color = read_image(color_path)
    depth_raw = read_image(depth_path)
    depth_m = depth_raw.astype(np.float32) / depth_scale
    # If "color" is really a grayscale/depth image, or doesn't match the depth
    # size (unaligned camera streams), draw on a depth picture instead.
    if color.ndim == 2 or color.shape[:2] != depth_m.shape:
        print("Color image missing or not aligned with depth - drawing on the depth image.")
        color = depth_to_color(depth_m)
    return color, depth_m, intrinsics


def backproject(depth_m, fx, fy, cx, cy, stride=1, max_depth=3.0):
    """Turn pixels (u, v) with depth Z into camera-frame points (X, Y, Z).

    Pinhole model:  X = (u - cx) * Z / fx,   Y = (v - cy) * Z / fy
    Camera frame:   +X right, +Y down, +Z forward (out of the lens).

    Returns the points plus the (v, u) pixel each point came from, so results
    can be painted back onto the image.
    """
    h, w = depth_m.shape
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    zs = depth_m[vs, us]
    valid = (zs > 0.1) & (zs < max_depth) & np.isfinite(zs)
    us, vs, zs = us[valid], vs[valid], zs[valid]
    xs = (us - cx) * zs / fx
    ys = (vs - cy) * zs / fy
    points = np.stack([xs, ys, zs], axis=1)
    pixels = np.stack([vs, us], axis=1)
    return points, pixels


# ---------------------------------------------------------------------------
# Step 2: find the table plane
# ---------------------------------------------------------------------------

def find_planes(points, max_planes=6, dist_thresh=0.02, min_inliers=1500, iterations=1000):
    """Peel off the biggest planes one at a time with RANSAC.

    Returns a list of (plane_model, inlier_indices) where plane_model is
    (a, b, c, d) for the plane a*x + b*y + c*z + d = 0.
    """
    remaining = np.arange(len(points))
    planes = []
    for _ in range(max_planes):
        if len(remaining) < min_inliers:
            break
        if o3d is not None:
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[remaining]))
            model, inliers = pcd.segment_plane(
                distance_threshold=dist_thresh, ransac_n=3, num_iterations=iterations
            )
        else:
            model, inliers = ransac_plane_numpy(points[remaining], dist_thresh, iterations)
        if len(inliers) < min_inliers:
            break
        planes.append((np.array(model), remaining[inliers]))
        remaining = np.delete(remaining, inliers)
    return planes


def drop_low_planes(planes, points, cam_to_base, min_height, verbose=True):
    """Skip planes whose points sit below min_height in the base frame (the floor).

    cam_to_base is (R, t) taking camera-frame points into the base frame, whose
    +z is up. Each plane's height is the median base-frame z of its points.
    """
    R, t = cam_to_base
    kept = []
    for model, idx in planes:
        height = float(np.median(points[idx] @ R[2] + t[2]))
        if height >= min_height:
            kept.append((model, idx))
        elif verbose:
            print(f"  plane: {len(idx):6d} pts at z={height:.2f} m -> below the "
                  f"{min_height:.2f} m minimum, skipped (floor?)")
    return kept


def pick_table(planes, up=(0.0, -1.0, 0.0), max_tilt_deg=40.0, max_dist=None, verbose=True):
    """Choose the table among the planes.

    - "Horizontal" = normal within max_tilt_deg of the up direction.
      In a camera frame +Y points down, so up is roughly -Y. On the robot you
      should use the real up direction from TF (base_link z-axis) instead.
    - Among horizontal planes, the table is the biggest one. (Not the closest:
      the flat top of a box on the table is closer and would win. The floor is
      kept out by max_dist here and min_height in drop_low_planes.)
    - If up is None (no TF available), just take the biggest plane. That works
      when the camera is pointed at the table.
    - Planes more than max_dist from the camera are skipped first, so the floor
      can't win even when it is the biggest plane in view.
    """
    if max_dist is not None:
        near = []
        for model, idx in planes:
            dist_to_camera = abs(model[3]) / np.linalg.norm(model[:3])
            if dist_to_camera <= max_dist:
                near.append((model, idx))
            elif verbose:
                print(f"  plane: {len(idx):6d} pts, {dist_to_camera:.2f} m from camera "
                      f"-> farther than {max_dist:.2f} m, skipped (floor?)")
        planes = near

    if up is None:
        if not planes:
            return None
        model, idx = max(planes, key=lambda p: len(p[1]))
        return model, idx, abs(model[3]) / np.linalg.norm(model[:3])

    up = np.asarray(up) / np.linalg.norm(up)
    best = None
    for model, idx in planes:
        normal = model[:3] / np.linalg.norm(model[:3])
        tilt = np.degrees(np.arccos(abs(normal @ up)))
        dist_to_camera = abs(model[3]) / np.linalg.norm(model[:3])
        horizontal = tilt <= max_tilt_deg
        if verbose:
            print(f"  plane: {len(idx):6d} pts, tilt {tilt:5.1f} deg, "
                  f"{dist_to_camera:.2f} m from camera -> {'horizontal' if horizontal else 'not horizontal'}")
        if horizontal and (best is None or len(idx) > len(best[1])):
            best = (model, idx, dist_to_camera)
    return best


# ---------------------------------------------------------------------------
# Step 3: describe the table and draw it
# ---------------------------------------------------------------------------

def table_mask(depth_m, intrinsics, model, dist_thresh=0.02, points=None, pixels=None):
    """Full-resolution mask: every pixel whose 3D point lies on the table plane.

    points/pixels let a caller hand in a back-projection it already computed.
    """
    if points is None:
        points, pixels = backproject(depth_m, **intrinsics, stride=1)
    normal_len = np.linalg.norm(model[:3])
    dist = np.abs(points @ model[:3] + model[3]) / normal_len
    on_plane = dist < dist_thresh
    mask = np.zeros(depth_m.shape, dtype=bool)
    mask[pixels[on_plane, 0], pixels[on_plane, 1]] = True

    # A plane is infinite, so anything that happens to cross the table's height
    # (chair backs, armrests) also lands on it. Keep only the biggest connected
    # blob of pixels - that's the actual table top.
    labels, n = ndimage.label(ndimage.binary_closing(mask, iterations=2) & mask)
    if n > 1:
        sizes = ndimage.sum(mask, labels, index=range(1, n + 1))
        mask = labels == (1 + int(np.argmax(sizes)))

    keep = mask[pixels[on_plane, 0], pixels[on_plane, 1]]
    return mask, points[on_plane][keep]


def describe_table(table_points):
    """Center and rough size of the table surface (camera frame, meters)."""
    center = table_points.mean(axis=0)
    lo, hi = np.percentile(table_points, [2, 98], axis=0)  # ignore stray points
    return center, hi - lo


# ---------------------------------------------------------------------------
# Step 4: find free space on the table for a bowl
# ---------------------------------------------------------------------------

# Biggest occupancy grid we will ever build. A 1.5 m table at 5 mm is only
# 300x300 cells; the cap is there so a bad plane fit can't ask for gigabytes.
MAX_GRID_CELLS = 1_000_000


def plane_axes(normal):
    """Two perpendicular unit vectors that lie in the plane with this normal.

    Which way they point doesn't matter - they just give us flat "x" and "y"
    directions to lay the top-down grid out along.
    """
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    x_axis = np.cross([0.0, 1.0, 0.0], normal)
    if np.linalg.norm(x_axis) < 1e-6:  # normal is parallel to y: pick another seed
        x_axis = np.cross([1.0, 0.0, 0.0], normal)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    return x_axis, y_axis


def view_center_on_plane(model, fallback):
    """Where the camera's optical axis (+Z out of the lens) pierces the table plane."""
    c, d = model[2], model[3]
    if abs(c) > 1e-9:
        z = -d / c
        if np.isfinite(z) and z > 0:
            return np.array([0.0, 0.0, z])
    return np.asarray(fallback, dtype=float)  # plane edge-on to the camera


def _placement_search(depth_m, intrinsics, table_result, bowl_radius=BOWL_RADIUS,
                      margin=MARGIN, cell_size=CELL_SIZE, edge_margin=EDGE_MARGIN,
                      surface_tol=0.02, obstacle_range=(0.01, 0.40),
                      points=None, pixels=None, allowed=None):
    """The worker behind find_placement().

    Returns (placement_or_None, reason, obstacle_mask). The mask comes back even
    when no placement fits, so the overlay can still show what is in the way.
    """
    model = np.asarray(table_result["model"], dtype=float)
    table_points = np.asarray(table_result["points"], dtype=float)
    if len(table_points) == 0:
        return None, "the table has no surface points", None

    # A frame sitting on the table: origin on the plane below the table's center,
    # ex/ey flat in the plane, normal pointing up out of it toward the camera.
    normal = np.asarray(table_result["normal"], dtype=float)
    unit = model[:3] / np.linalg.norm(model[:3])
    offset = model[3] / np.linalg.norm(model[:3])
    center = np.asarray(table_result["center"], dtype=float)
    origin = center - (center @ unit + offset) * unit
    ex, ey = plane_axes(normal)

    # Only count surface points that really are on the plane.
    table_points = table_points[np.abs((table_points - origin) @ normal) <= surface_tol]
    if len(table_points) == 0:
        return None, "the table has no surface points", None

    # Grid bounds: the table's own extent plus a ring of padding, so that the
    # table edge is surrounded by UNKNOWN (= blocked) cells rather than by the
    # end of the array, which the distance transform would treat as free.
    ta = (table_points - origin) @ ex
    tb = (table_points - origin) @ ey
    pad = bowl_radius + max(margin, edge_margin) + 4 * cell_size
    a0, b0 = ta.min() - pad, tb.min() - pad
    a1, b1 = ta.max() + pad, tb.max() + pad
    cell = max(cell_size, np.sqrt(max((a1 - a0) * (b1 - b0), 1e-6) / MAX_GRID_CELLS))
    na = int(np.ceil((a1 - a0) / cell)) + 1
    nb = int(np.ceil((b1 - b0) / cell)) + 1

    # OCCUPIED: anything standing on the table, from ALL depth points.
    if points is None:
        points, pixels = backproject(depth_m, **intrinsics, stride=1)
    rel = points - origin
    height = rel @ normal  # positive = camera side of the plane
    ia = np.floor((rel @ ex - a0) / cell).astype(np.intp)
    ib = np.floor((rel @ ey - b0) / cell).astype(np.intp)
    low, high = obstacle_range
    is_obstacle = ((ia >= 0) & (ia < na) & (ib >= 0) & (ib < nb)
                   & (height > low) & (height < high))
    occupied = np.zeros((na, nb), dtype=bool)
    occupied[ia[is_obstacle], ib[is_obstacle]] = True
    obstacle_mask = np.zeros(depth_m.shape, dtype=bool)
    obstacle_mask[pixels[is_obstacle, 0], pixels[is_obstacle, 1]] = True

    # FREE: cells where we actually saw bare tabletop and nothing above it.
    fa = np.floor((ta - a0) / cell).astype(np.intp)
    fb = np.floor((tb - b0) / cell).astype(np.intp)
    free = np.zeros((na, nb), dtype=bool)
    free[fa, fb] = True
    free &= ~occupied  # everything else (OCCUPIED and UNKNOWN alike) blocks the bowl

    # Split the blocked cells into two kinds, because they get different gaps:
    #   inside the table's outline (objects, their blind-spot shadows, depth holes)
    #   -> OBSTACLE, keep `margin`;  outside the outline (past the edge) -> EDGE,
    #   keep `edge_margin`. The outline is the seen tabletop with its interior holes
    #   filled in (closing first bridges tiny gaps so the outline stays one piece).
    table_region = ndimage.binary_fill_holes(ndimage.binary_closing(free, iterations=2))
    table_region |= free
    obstacle_cells = table_region & ~free

    # Distance from every cell to the nearest edge / obstacle cell.
    dist_edge = ndimage.distance_transform_edt(table_region, sampling=cell)
    if obstacle_cells.any():
        dist_obstacle = ndimage.distance_transform_edt(~obstacle_cells, sampling=cell)
    else:
        dist_obstacle = np.full(free.shape, np.inf)
    clearance = np.where(free, np.minimum(dist_edge, dist_obstacle), 0.0)

    need_edge = bowl_radius + edge_margin
    need_obstacle = bowl_radius + margin
    fits_edge = free & (dist_edge >= need_edge)
    fits_obstacle = free & (dist_obstacle >= need_obstacle)
    valid = fits_edge & fits_obstacle
    if allowed is not None and valid.any():
        # Caller's extra rule (e.g. "the arm can reach it"), on camera-frame cell centres.
        ja, jb = np.nonzero(valid)
        centres = origin + (a0 + (ja[:, None] + 0.5) * cell) * ex + (b0 + (jb[:, None] + 0.5) * cell) * ey
        keep = np.asarray(allowed(centres), dtype=bool)
        if not keep.any():
            return None, "the clear spots are all outside the allowed area (e.g. out of reach)", obstacle_mask
        valid = np.zeros_like(valid)
        valid[ja[keep], jb[keep]] = True
    if not valid.any():
        if not fits_edge.any():
            best = float(np.where(free, dist_edge, 0).max())
            why = (f"no spot is {need_edge * 100:.1f} cm from the table edge (best has "
                   f"{best * 100:.1f} cm; bowl radius {bowl_radius * 100:.1f} cm + edge "
                   f"gap {edge_margin * 100:.1f} cm) - the visible table is too small or "
                   f"cluttered near its edges; aim the camera at more open table")
        elif not fits_obstacle.any():
            best = float(np.where(free, dist_obstacle, 0).max())
            why = (f"nothing on the table is {need_obstacle * 100:.1f} cm clear of the "
                   f"nearest object (best spot has {best * 100:.1f} cm; bowl radius "
                   f"{bowl_radius * 100:.1f} cm + margin {margin * 100:.1f} cm)")
        else:
            why = (f"the spots far enough from the edge ({need_edge * 100:.1f} cm) are all "
                   f"within {need_obstacle * 100:.1f} cm of an object - clear some space")
        # Far away, the depth points are spread further apart than the grid cells,
        # so the free area comes out speckled with holes that look like obstacles.
        if free.any() and len(table_points) < 2 * free.sum():
            why += (f" - only {len(table_points) / free.sum():.1f} depth points per "
                    f"{cell * 1000:.0f} mm cell, so try a bigger cell size")
        return None, why, obstacle_mask

    # Of the spots that fit, take the one nearest the middle of the camera's view.
    aim = view_center_on_plane(model, center)
    aim_a, aim_b = (aim - origin) @ ex, (aim - origin) @ ey
    ja, jb = np.nonzero(valid)
    ca = a0 + (ja + 0.5) * cell
    cb = b0 + (jb + 0.5) * cell
    k = int(np.argmin((ca - aim_a) ** 2 + (cb - aim_b) ** 2))
    point = origin + ca[k] * ex + cb[k] * ey

    return dict(point=point, radius=float(bowl_radius), margin=float(margin),
                edge_margin=float(edge_margin),
                clearance=float(clearance[ja[k], jb[k]]),
                edge_clearance=float(dist_edge[ja[k], jb[k]]),
                obstacle_clearance=float(dist_obstacle[ja[k], jb[k]]), normal=normal,
                axes=(ex, ey), origin=origin, cell_size=float(cell), aim=aim,
                free=free, occupied=occupied, obstacle_mask=obstacle_mask,
                grid_origin=(float(a0), float(b0))), "", obstacle_mask


def find_placement(depth_m, intrinsics, table_result, bowl_radius=BOWL_RADIUS, margin=MARGIN,
                   cell_size=CELL_SIZE, edge_margin=EDGE_MARGIN, verbose=False, **kwargs):
    """Find where a bowl of radius bowl_radius can be set down on the table.

    table_result is what detect_table() returned. Returns None if there is no
    room (pass verbose=True to have the reason printed), otherwise a dict with:
      point        - center of the free circle, camera frame (m), on the plane
      radius       - the bowl radius that was asked for (m)
      clearance    - actual distance from that point to the nearest blocked cell (m)
      edge_clearance / obstacle_clearance - the same, to the table edge / nearest object
      axes         - the two in-plane unit vectors the circle lives in
      obstacle_mask- HxW bool, pixels of the things standing on the table
      free/occupied- the top-down grids, mostly for debugging and tests
    """
    placement, reason, _ = _placement_search(
        depth_m, intrinsics, table_result, bowl_radius=bowl_radius, margin=margin,
        cell_size=cell_size, edge_margin=edge_margin, **kwargs)
    if placement is None and verbose:
        print(f"No room for the bowl: {reason}")
    return placement


def detect_table(depth_m, intrinsics, up=(0.0, -1.0, 0.0), verbose=False, fast=False,
                 bowl_radius=BOWL_RADIUS, margin=MARGIN, cell_size=CELL_SIZE,
                 edge_margin=EDGE_MARGIN, placement=True, max_plane_dist=None,
                 cam_to_base=None, min_height=None, allowed=None):
    """Whole pipeline in one call. Used by both this script and the ROS node.

    Returns None if no table, else a dict with:
      model  - plane (a, b, c, d), camera frame
      normal - unit normal pointing toward the camera
      mask   - HxW bool array of table pixels
      points - Nx3 table points, camera frame (m)
      center - table center, camera frame (m)
      size   - extent along x, y, z (m)
      placement        - find_placement() result, or None if the bowl doesn't fit
      placement_reason - why there is no placement ("" when there is one)
      obstacle_mask    - HxW bool, pixels of the things standing on the table
    fast=True trades a little accuracy for ~4x speed.
    placement=False skips the free-space search.
    max_plane_dist (m) skips planes farther than that from the camera.
    min_height (m) skips planes below that height in the base frame; it needs
    cam_to_base = (R, t) and is ignored without it.
    allowed(points Nx3, camera frame) -> bool mask limits where the bowl may go.
    """
    # fast=True: sample every 4th pixel and do fewer RANSAC tries (for the Jetson).
    stride = 4 if fast else 2
    points, _ = backproject(depth_m, **intrinsics, stride=stride)
    planes = find_planes(points, min_inliers=1500 // stride ** 2 * 4,
                         iterations=300 if fast else 1000)
    if verbose:
        print("Planes found:")
    if min_height is not None and cam_to_base is not None:
        planes = drop_low_planes(planes, points, cam_to_base, min_height, verbose=verbose)
    table = pick_table(planes, up=up, max_dist=max_plane_dist, verbose=verbose)
    if table is None:
        return None
    model, _, dist = table
    # One full-resolution back-projection, shared by the mask and the bowl search.
    full_points, full_pixels = backproject(depth_m, **intrinsics, stride=1)
    mask, table_points = table_mask(depth_m, intrinsics, model,
                                    points=full_points, pixels=full_pixels)
    if len(table_points) == 0:
        return None
    center, size = describe_table(table_points)
    normal = model[:3] / np.linalg.norm(model[:3])
    if normal @ (-center) < 0:  # flip so it points from the table toward the camera
        normal = -normal
    result = dict(model=model, normal=normal, mask=mask, points=table_points,
                  center=center, size=size, dist=dist,
                  placement=None, placement_reason="", obstacle_mask=None)
    if placement:
        spot, reason, obstacles = _placement_search(
            depth_m, intrinsics, result, bowl_radius=bowl_radius, margin=margin,
            cell_size=cell_size, edge_margin=edge_margin,
            points=full_points, pixels=full_pixels, allowed=allowed)
        result["placement"] = spot
        result["placement_reason"] = reason
        result["obstacle_mask"] = obstacles
        if verbose and spot is None:
            print(f"No room for the bowl: {reason}")
    return result


def project_to_pixels(points, fx, fy, cx, cy):
    """Camera-frame 3D points -> (u, v) pixels, plus a flag per point for
    "actually in front of the lens" (points behind it can't be drawn)."""
    points = np.atleast_2d(np.asarray(points, dtype=float))
    z = points[:, 2]
    in_front = z > 1e-4
    safe = np.where(in_front, z, 1.0)
    uv = np.stack([points[:, 0] * fx / safe + cx, points[:, 1] * fy / safe + cy], axis=1)
    return uv, in_front


def _draw_polyline(img, uv, valid, color, width=2):
    """Join uv[0]->uv[1]->... with straight lines, skipping any segment whose
    endpoint is behind the camera. Plain NumPy, so no OpenCV needed."""
    h, w = img.shape[:2]
    us, vs = [], []
    for i in range(len(uv) - 1):
        if not (valid[i] and valid[i + 1]):
            continue
        (u0, v0), (u1, v1) = uv[i], uv[i + 1]
        steps = min(int(max(abs(u1 - u0), abs(v1 - v0))) + 1, 4000)
        t = np.linspace(0.0, 1.0, steps + 1)
        us.append(u0 + (u1 - u0) * t)
        vs.append(v0 + (v1 - v0) * t)
    if not us:
        return
    u, v = np.round(np.concatenate(us)), np.round(np.concatenate(vs))
    off = np.arange(width) - width // 2  # square brush, so the line is visible
    du, dv = np.meshgrid(off, off)
    uu = (u[:, None] + du.ravel()).ravel().astype(np.intp)
    vv = (v[:, None] + dv.ravel()).ravel().astype(np.intp)
    keep = (uu >= 0) & (uu < w) & (vv >= 0) & (vv < h)
    img[vv[keep], uu[keep]] = np.asarray(color, dtype=img.dtype)


def draw_placement(img, placement, intrinsics, color=(0, 255, 0), width=2, samples=96):
    """Draw the bowl footprint: its circle on the table plus a cross at the center."""
    ex, ey = placement["axes"]
    center, radius = placement["point"], placement["radius"]
    ang = np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False)
    ring = center + radius * (np.cos(ang)[:, None] * ex + np.sin(ang)[:, None] * ey)
    uv, ok = project_to_pixels(np.vstack([ring, ring[:1]]), **intrinsics)
    _draw_polyline(img, uv, ok, color, width)
    for axis in (ex, ey):  # little cross marking the center
        uv, ok = project_to_pixels(
            np.stack([center - 0.01 * axis, center + 0.01 * axis]), **intrinsics)
        _draw_polyline(img, uv, ok, color, width)


def overlay_image(color, mask, alpha=0.55, obstacle_mask=None, placement=None,
                  intrinsics=None):
    """Color image (HxWx3 uint8) with the table in red, whatever is standing on
    it in yellow, and the chosen bowl spot as a green circle."""
    out = color[..., :3].astype(np.float32).copy()
    red = np.array([255.0, 0.0, 0.0])
    out[mask] = (1 - alpha) * out[mask] + alpha * red
    if obstacle_mask is not None:
        yellow = np.array([255.0, 255.0, 0.0])
        out[obstacle_mask] = (1 - alpha) * out[obstacle_mask] + alpha * yellow
    out = np.ascontiguousarray(out.astype(np.uint8))
    if placement is not None and intrinsics is not None:
        draw_placement(out, placement, intrinsics)
    return out


def depth_to_color(depth_m, max_depth=3.0):
    """Grayscale picture of a depth image (near = bright), for when color isn't aligned."""
    d = np.clip(depth_m / max_depth, 0, 1)
    gray = ((1 - d) * 255 * (depth_m > 0)).astype(np.uint8)
    return np.stack([gray] * 3, axis=-1)


def save_overlay(color, mask, path, alpha=0.55, **kwargs):
    """Paint the table (and the bowl spot) on the color image and save it."""
    write_image(path, overlay_image(color, mask, alpha, **kwargs))


def show_3d(points, table_points):
    """Interactive 3D view (gray scene, red table). Needs a screen and Open3D."""
    if o3d is None:
        print("--show needs Open3D (pip install open3d)")
        return
    scene = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    scene.paint_uniform_color([0.6, 0.6, 0.6])
    table = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(table_points))
    table.paint_uniform_color([1.0, 0.0, 0.0])
    o3d.visualization.draw_geometries([scene, table])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--color")
    parser.add_argument("--depth")
    parser.add_argument("--depth-scale", type=float, default=1000.0)
    parser.add_argument("--fx", type=float)
    parser.add_argument("--fy", type=float)
    parser.add_argument("--cx", type=float)
    parser.add_argument("--cy", type=float)
    parser.add_argument("--out", default="table_overlay.png")
    parser.add_argument("--show", action="store_true", help="open a 3D viewer (needs a screen)")
    parser.add_argument("--largest-plane", action="store_true",
                        help="skip the 'horizontal' check and take the biggest plane "
                             "(use for wrist-camera frames looking down at the table)")
    parser.add_argument("--max-plane-dist", type=float, default=MAX_PLANE_DIST,
                        help="skip planes farther than this from the camera, in m "
                             f"(default {MAX_PLANE_DIST:.2f}; ignored on the sample frame)")
    args = parser.parse_args()

    if args.color and args.depth:
        intr = dict(fx=args.fx, fy=args.fy, cx=args.cx, cy=args.cy)
        color, depth_m, intr = load_frame(args.color, args.depth, args.depth_scale, intr)
        max_plane_dist = args.max_plane_dist
    else:
        color, depth_m, intr = load_sample_frame()
        max_plane_dist = None  # the sample's desk is farther away than our robot's table

    points, _ = backproject(depth_m, **intr, stride=2)
    print(f"Image {color.shape[1]}x{color.shape[0]}, {len(points)} valid 3D points")
    print(f"Depth range: {points[:, 2].min():.2f} m to {points[:, 2].max():.2f} m")

    up = None if args.largest_plane else (0.0, -1.0, 0.0)
    # Bowl size and the safety gaps are fixed at the top of this file.
    result = detect_table(depth_m, intr, up=up, verbose=True, max_plane_dist=max_plane_dist)
    if result is None:
        print("No table found.")
        return
    mask, table_points = result["mask"], result["points"]
    center, size = result["center"], result["size"]
    print(f"Table: {result['dist']:.2f} m from camera, "
          f"plane {np.round(result['model'], 3).tolist()}")
    print(f"Table center (camera frame): x={center[0]:.2f} y={center[1]:.2f} z={center[2]:.2f} m")
    print(f"Table extent: {size[0]:.2f} m wide (x), {size[2]:.2f} m deep (z)")
    print(f"Table covers {mask.mean() * 100:.1f}% of the image")

    placement = result["placement"]
    if placement is None:
        print(f"No room for a {BOWL_RADIUS * 100:.1f} cm bowl: {result['placement_reason']}")
    else:
        p = placement["point"]
        print(f"Bowl placement (camera frame): x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} m, "
              f"circle radius {placement['radius'] * 100:.1f} cm "
              f"({placement['obstacle_clearance'] * 100:.1f} cm from the nearest object, "
              f"{placement['edge_clearance'] * 100:.1f} cm from the table edge)")

    save_overlay(color, mask, args.out, placement=placement, intrinsics=intr,
                 obstacle_mask=result["obstacle_mask"])
    print(f"Saved {args.out}")
    if args.show:
        show_3d(points, table_points)


if __name__ == "__main__":
    main()
