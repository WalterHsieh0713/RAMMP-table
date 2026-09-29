"""
Tests for the bowl-placement search in table_detect.py - no ROS, no robot, no camera.

Everything runs on synthetic depth images that we ray-trace ourselves: a flat
rectangular table seen by a camera tilted ~45 degrees down from 0.5 m away, with
a few box-shaped objects sitting on it. Because we built the scene, we know
exactly where the table edge and every box really are, so we can check the
placement against the truth instead of against the detector's own grid.

    python test_placement.py          (or: pytest -q test_placement.py)
"""

import time

import numpy as np

from table_detect import detect_table, find_placement


# ---------------------------------------------------------------------------
# A tiny depth-image renderer
# ---------------------------------------------------------------------------

class Scene:
    """A table + boxes, and the depth image a tilted camera would see of them.

    Boxes are given in table coordinates as (a_center, b_center, a_size, b_size,
    height): "a" runs across the table (camera right), "b" runs away from the
    camera, height is straight up off the tabletop. All meters.
    """

    def __init__(self, boxes=(), tilt_deg=45.0, distance=0.5,
                 a_range=(-0.22, 0.22), b_range=(-0.15, 0.40),
                 width=640, height=480, fx=615.0, fy=615.0):
        self.boxes = [tuple(float(v) for v in box) for box in boxes]
        self.a_range, self.b_range = a_range, b_range
        self.intrinsics = dict(fx=fx, fy=fy, cx=(width - 1) / 2.0, cy=(height - 1) / 2.0)
        self.shape = (height, width)

        # Table frame in camera coordinates (+X right, +Y down, +Z forward).
        # Tilting the camera down by t puts "up" between -Y (level) and -Z (straight down).
        t = np.radians(tilt_deg)
        self.normal = np.array([0.0, -np.cos(t), -np.sin(t)])   # table up, toward the camera
        self.ex = np.array([1.0, 0.0, 0.0])                     # across the table
        self.ey = np.cross(self.normal, self.ex)                # away from the camera
        self.origin = np.array([0.0, 0.0, distance])            # where the optical axis lands

    # -- geometry helpers ---------------------------------------------------

    def to_table(self, points):
        """Camera-frame points -> true (a, b, height-above-table) coordinates."""
        rel = np.atleast_2d(np.asarray(points, dtype=float)) - self.origin
        return np.stack([rel @ self.ex, rel @ self.ey, rel @ self.normal], axis=1)

    def on_table(self, a, b):
        """Is (a, b) inside the real tabletop rectangle?"""
        return ((a >= self.a_range[0]) & (a <= self.a_range[1])
                & (b >= self.b_range[0]) & (b <= self.b_range[1]))

    def in_box(self, a, b):
        """Is (a, b) inside the footprint of any object?"""
        hit = np.zeros(np.shape(a), dtype=bool)
        for ca, cb, sa, sb, _h in self.boxes:
            hit |= (np.abs(a - ca) <= sa / 2) & (np.abs(b - cb) <= sb / 2)
        return hit

    def edge_distance(self, a, b):
        """True distance from (a, b) to the nearest edge of the tabletop."""
        return min(a - self.a_range[0], self.a_range[1] - a,
                   b - self.b_range[0], self.b_range[1] - b)

    def box_distance(self, a, b):
        """True distance from (a, b) to the nearest box footprint (inf if no boxes)."""
        best = np.inf
        for ca, cb, sa, sb, _h in self.boxes:
            da = max(abs(a - ca) - sa / 2, 0.0)
            db = max(abs(b - cb) - sb / 2, 0.0)
            best = min(best, float(np.hypot(da, db)))
        return best

    def clearance(self, a, b):
        """True distance from (a, b) to the table edge or nearest box, whichever is closer."""
        best = min(a - self.a_range[0], self.a_range[1] - a,
                   b - self.b_range[0], self.b_range[1] - b)
        for ca, cb, sa, sb, _h in self.boxes:
            da = max(abs(a - ca) - sa / 2, 0.0)
            db = max(abs(b - cb) - sb / 2, 0.0)
            best = min(best, float(np.hypot(da, db)))
        return best

    # -- rendering ----------------------------------------------------------

    def depth(self):
        """Ray-trace one depth image (meters, 0 where the ray hit nothing)."""
        h, w = self.shape
        i = self.intrinsics
        vs, us = np.mgrid[0:h, 0:w]
        # Rays out of the pinhole, scaled so their z component is exactly 1 -
        # then the ray parameter t IS the depth in meters.
        d_cam = np.stack([(us.ravel() - i["cx"]) / i["fx"],
                          (vs.ravel() - i["cy"]) / i["fy"],
                          np.ones(h * w)], axis=1)
        basis = np.column_stack([self.ex, self.ey, self.normal])
        d = d_cam @ basis                                      # ray direction, table coords
        o = np.broadcast_to((-self.origin) @ basis, d.shape)   # camera position, table coords

        best = np.full(h * w, np.inf)

        # The tabletop: where the ray crosses height 0, inside the rectangle.
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -o[:, 2] / d[:, 2]
        a, b = o[:, 0] + t * d[:, 0], o[:, 1] + t * d[:, 1]
        ok = np.isfinite(t) & (t > 0) & self.on_table(a, b)
        best = np.where(ok, np.minimum(best, t), best)

        # The boxes: the usual slab test against an axis-aligned block.
        for ca, cb, sa, sb, bh in self.boxes:
            lo = np.array([ca - sa / 2, cb - sb / 2, 0.0])
            hi = np.array([ca + sa / 2, cb + sb / 2, bh])
            with np.errstate(divide="ignore", invalid="ignore"):
                t1, t2 = (lo - o) / d, (hi - o) / d
            near = np.nanmax(np.minimum(t1, t2), axis=1)
            far = np.nanmin(np.maximum(t1, t2), axis=1)
            t = np.where(near > 0, near, far)
            ok = (far >= np.maximum(near, 0.0)) & (t > 0)
            best = np.where(ok, np.minimum(best, t), best)

        depth = np.where(np.isfinite(best), best, 0.0)
        return depth.reshape(h, w).astype(np.float32)

    # -- running the detector ----------------------------------------------

    def detect(self, **kwargs):
        """detect_table() on this scene, checking first that it found OUR table."""
        result = detect_table(self.depth(), self.intrinsics, up=None, **kwargs)
        assert result is not None, "the detector did not find the synthetic table at all"
        tilt = np.degrees(np.arccos(np.clip(abs(result["normal"] @ self.normal), -1, 1)))
        assert tilt < 3.0, f"detected plane is tilted {tilt:.1f} deg away from the real table"
        height = self.to_table(result["center"])[0, 2]
        assert abs(height) < 0.01, f"detected plane sits {height * 100:.1f} cm off the real table"
        return result


# ---------------------------------------------------------------------------
# Shared checks
# ---------------------------------------------------------------------------

def check_circle_is_clear(scene, placement, label=""):
    """Every point of the bowl's footprint must land on bare table."""
    center, radius = placement["point"], placement["radius"]
    ex, ey = placement["axes"]

    a0, b0, h0 = scene.to_table(center)[0]
    assert abs(h0) < 0.01, f"{label}placement is {h0 * 100:.1f} cm off the table plane"
    assert scene.on_table(a0, b0), f"{label}placement center is off the table"

    # The whole disk, not just the rim: rings at 25/50/75/100% of the radius
    # would also catch a small object swallowed by the circle.
    angles = np.linspace(0, 2 * np.pi, 180, endpoint=False)
    for frac in (0.25, 0.5, 0.75, 1.0):
        ring = center + radius * frac * (np.cos(angles)[:, None] * ex
                                         + np.sin(angles)[:, None] * ey)
        a, b, _ = scene.to_table(ring).T
        off = ~scene.on_table(a, b)
        assert not off.any(), (f"{label}{off.sum()} of {len(a)} points at {frac:.0%} "
                               "radius fall off the table edge")
        inside = scene.in_box(a, b)
        assert not inside.any(), (f"{label}{inside.sum()} of {len(a)} points at {frac:.0%} "
                                  "radius are inside an object")

    true_clear = scene.clearance(a0, b0)
    assert true_clear >= radius, (f"{label}only {true_clear * 100:.1f} cm of real clearance "
                                  f"for a {radius * 100:.1f} cm bowl")
    return true_clear


def three_box_scene():
    return Scene(boxes=[
        (-0.13, 0.05, 0.07, 0.07, 0.09),   # near left
        (0.12, 0.28, 0.08, 0.06, 0.06),    # far right
        (-0.02, 0.33, 0.06, 0.09, 0.12),   # far middle, tall
    ])


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_placement_is_clear_of_objects_and_edges():
    scene = three_box_scene()
    result = scene.detect()
    placement = result["placement"]
    assert placement is not None, f"no placement found: {result['placement_reason']}"
    clear = check_circle_is_clear(scene, placement)
    print(f"  placement {np.round(placement['point'], 3).tolist()} m, "
          f"reported clearance {placement['clearance'] * 100:.1f} cm, "
          f"true clearance {clear * 100:.1f} cm")


def test_placement_avoids_objects_in_several_layouts():
    scenes = [
        three_box_scene(),
        Scene(boxes=[(0.0, 0.12, 0.10, 0.10, 0.08)]),           # one box dead center
        Scene(boxes=[(-0.10, 0.10, 0.12, 0.12, 0.05),
                     (0.10, 0.10, 0.12, 0.12, 0.15)]),          # left and right
        Scene(boxes=[(0.0, 0.00, 0.30, 0.05, 0.05),
                     (0.0, 0.38, 0.30, 0.05, 0.10)]),           # a wall in front and behind
    ]
    for i, scene in enumerate(scenes):
        # These 44 cm synthetic tables are too small for the default 8 cm edge gap
        # once boxes are added, so this test (about dodging objects) uses 2 cm everywhere.
        result = scene.detect(edge_margin=0.02)
        placement = result["placement"]
        assert placement is not None, f"scene {i}: no placement ({result['placement_reason']})"
        check_circle_is_clear(scene, placement, label=f"scene {i}: ")


def test_crowded_table_has_no_placement():
    """Boxes every 12 cm leave ~6 cm gaps - nowhere near enough for a 15 cm bowl."""
    boxes = []
    for k, a in enumerate(np.arange(-0.18, 0.19, 0.12)):
        for j, b in enumerate(np.arange(-0.08, 0.39, 0.12)):
            boxes.append((a, b, 0.06, 0.06, 0.05 + 0.02 * ((k + j) % 4)))
    scene = Scene(boxes=boxes)
    result = scene.detect()
    assert result["placement"] is None, (
        "found a placement on a crowded table at "
        f"{np.round(result['placement']['point'], 3).tolist() if result['placement'] else ''}")
    assert result["placement_reason"], "returned None without saying why"
    print(f"  crowded table: {result['placement_reason']}")


def test_bare_table_places_near_the_center_of_view():
    """With nothing in the way the bowl should land close to where the camera looks."""
    scene = Scene()
    result = scene.detect()
    placement = result["placement"]
    assert placement is not None, result["placement_reason"]
    check_circle_is_clear(scene, placement, label="bare table: ")
    a, b, _ = scene.to_table(placement["point"])[0]
    # The optical axis hits the table at (a, b) = (0, 0), and that point is more
    # than a bowl radius from every edge, so that is exactly where it should go.
    assert np.hypot(a, b) < 0.02, f"bowl went to (a={a:.3f}, b={b:.3f}), not the view center"


def test_bigger_bowl_moves_the_placement_or_gives_up():
    scene = three_box_scene()
    depth = scene.depth()
    small = scene.detect(bowl_radius=0.06)["placement"]
    assert small is not None, "no room even for a small bowl"
    check_circle_is_clear(scene, small, label="r=6cm: ")

    table = scene.detect(placement=False)
    changed, notes = False, []
    for radius in (0.10, 0.14, 0.18, 0.22):
        big = find_placement(depth, scene.intrinsics, table, bowl_radius=radius)
        if big is None:
            notes.append(f"r={radius * 100:.0f}cm: no room")
            changed = True
            continue
        check_circle_is_clear(scene, big, label=f"r={radius * 100:.0f}cm: ")
        assert big["clearance"] >= radius, "reported clearance is smaller than the bowl"
        shift = float(np.linalg.norm(big["point"] - small["point"]))
        notes.append(f"r={radius * 100:.0f}cm: moved {shift * 100:.1f} cm")
        changed = changed or shift > 1e-6
    print("  " + "; ".join(notes))
    assert changed, "growing the bowl neither moved the placement nor ruled it out"

    # A bowl wider than the table can never fit.
    assert find_placement(depth, scene.intrinsics, table, bowl_radius=0.40) is None, \
        "an 80 cm-wide bowl should not fit on a 44 cm-wide table"


def test_margin_is_respected():
    """A bigger margin must never leave the bowl sitting closer to something."""
    scene = three_box_scene()
    depth = scene.depth()
    table = scene.detect(placement=False)
    tight = find_placement(depth, scene.intrinsics, table, bowl_radius=0.06, margin=0.0)
    loose = find_placement(depth, scene.intrinsics, table, bowl_radius=0.06, margin=0.06)
    assert tight is not None and loose is not None
    for placement, name in ((tight, "margin=0"), (loose, "margin=6cm")):
        check_circle_is_clear(scene, placement, label=f"{name}: ")
    a, b, _ = scene.to_table(loose["point"])[0]
    # Allow a cell or two of grid rounding on top of the 12 cm we asked for.
    assert scene.clearance(a, b) >= 0.12 - 0.01, "the 6 cm margin was not honored"


def test_unknown_space_is_treated_as_blocked():
    """The shadow an object casts carries no depth data - the bowl must not go there."""
    scene = Scene(boxes=[(0.0, 0.0, 0.05, 0.05, 0.25)])  # tall and narrow, hides a wedge
    result = scene.detect(edge_margin=0.02)  # small table: test the shadow, not the edge gap
    placement = result["placement"]
    assert placement is not None, result["placement_reason"]
    check_circle_is_clear(scene, placement, label="shadow: ")

    # Every grid cell the bowl covers must be one we actually saw as free table.
    free, cell = placement["free"], placement["cell_size"]
    a0, b0 = placement["grid_origin"]
    ex, ey = placement["axes"]
    angles = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    for frac in (0.0, 0.5, 1.0):
        pts = placement["point"] + placement["radius"] * frac * (
            np.cos(angles)[:, None] * ex + np.sin(angles)[:, None] * ey)
        rel = pts - placement["origin"]
        ia = np.floor((rel @ ex - a0) / cell).astype(int)
        ib = np.floor((rel @ ey - b0) / cell).astype(int)
        assert free[ia, ib].all(), "the bowl covers cells never seen as free table"


def test_edge_margin_keeps_the_bowl_away_from_the_edge():
    """The camera looks at a spot only 5 cm from the table's left edge. The bowl must
    still land at least radius + edge_margin (5.5 + 8 cm) from every edge."""
    scene = Scene(a_range=(-0.05, 0.40), b_range=(-0.20, 0.40))
    result = scene.detect()  # defaults: 5.5 cm bowl, 8 cm edge gap
    placement = result["placement"]
    assert placement is not None, result["placement_reason"]
    check_circle_is_clear(scene, placement, label="edge gap: ")
    a, b, _ = scene.to_table(placement["point"])[0]
    edge = scene.edge_distance(a, b)
    need = placement["radius"] + placement["edge_margin"]
    print(f"  bowl center {edge * 100:.1f} cm from the edge (needs >= {need * 100:.1f} cm)")
    assert edge >= need - 0.01, f"bowl center only {edge * 100:.1f} cm from the table edge"

    # With a small edge gap it is allowed to sit much closer to the edge.
    close = scene.detect(edge_margin=0.0)["placement"]
    a2, b2, _ = scene.to_table(close["point"])[0]
    assert scene.edge_distance(a2, b2) < edge - 0.03, "edge_margin had no effect"


def test_objects_only_need_the_small_margin():
    """A box at the view center: the bowl should sit right next to it (radius + 2 cm),
    not pushed out to the 8 cm edge gap."""
    scene = Scene(boxes=[(0.0, 0.10, 0.08, 0.08, 0.06)],
                  a_range=(-0.35, 0.35), b_range=(-0.25, 0.45))
    result = scene.detect()
    placement = result["placement"]
    assert placement is not None, result["placement_reason"]
    check_circle_is_clear(scene, placement, label="object gap: ")
    a, b, _ = scene.to_table(placement["point"])[0]
    gap = scene.box_distance(a, b) - placement["radius"]
    print(f"  bowl rim {gap * 100:.1f} cm from the box (margin {placement['margin'] * 100:.0f} cm)")
    assert gap >= placement["margin"] - 0.01, "bowl is closer to the box than the margin"
    assert gap < placement["edge_margin"] - 0.01, "objects are being given the big edge gap"


def test_fast_mode_is_quick_enough():
    scene = three_box_scene()
    depth = scene.depth()
    detect_table(depth, scene.intrinsics, up=None, fast=True)  # warm up numpy/scipy
    times, result = [], None
    for _ in range(3):
        t0 = time.perf_counter()
        result = detect_table(depth, scene.intrinsics, up=None, fast=True)
        times.append(time.perf_counter() - t0)
    best = min(times)
    print(f"  fast mode: {best * 1000:.0f} ms per 640x480 frame (target < 300 ms)")
    assert result["placement"] is not None
    assert best < 1.0, f"fast mode took {best * 1000:.0f} ms, far over the 300 ms target"


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failures = 0
    for fn in tests:
        print(f"{fn.__name__} ...")
        try:
            fn()
        except AssertionError as e:
            failures += 1
            print(f"  FAIL: {e}")
        else:
            print("  ok")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
