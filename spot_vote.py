"""Many scans -> one bowl spot, and the click-to-confirm window (used by place_bowl.py look).

Pure numpy/scipy except the window itself (OpenCV, imported only when shown), so the
averaging and the checks run offline: python3 test_spot_vote.py

combine_spots   the spot found in each frame -> drop the outliers, average the rest, or refuse
grid_clearance  is the averaged spot still clear in one frame's free-space grid?
confirm_image   the picture: table red, objects yellow, every frame's spot, the final circle, buttons
ask             Confirm / Rescan / Cancel, by mouse or keys; a terminal prompt when there's no display
"""
import os
import sys

import numpy as np
from scipy import ndimage

from table_detect import draw_placement, _draw_polyline, project_to_pixels

MIN_FOUND = 3           # frames that must find a spot at all
MIN_INLIER_FRAC = 0.6   # ... and this share of them must agree
OUTLIER_TOL = 0.015     # m: never call a spot an outlier closer than this to the median ...
OUTLIER_MAD = 3.0       # ... else outlier = more than this x the median distance from the median
OUTLIER_CAP = 0.03      # m: ... but never accept one farther than this. Without the cap, two
                        # equal clusters 10 cm apart would all count as inliers and average onto
                        # the gap between them (which may be the object they're on either side of).


def combine_spots(points, min_found=MIN_FOUND, min_inlier_frac=MIN_INLIER_FRAC,
                  tol=OUTLIER_TOL, mad_k=OUTLIER_MAD, cap=OUTLIER_CAP):
    """points: Nx2 or Nx3, one spot per frame that found one (x, y used for the distances).

    Returns a dict:
      ok       True if the frames agree; reason says why not otherwise
      spot     mean of the inliers (all columns), or None
      median   per-axis median of x, y
      inliers  bool per point
      threshold  outlier distance used (m)
      spread   RMS distance of the inliers from the spot (m)
    """
    pts = np.asarray(points, dtype=float)
    pts = np.atleast_2d(pts) if pts.size else np.zeros((0, 2))
    out = dict(ok=False, reason="", spot=None, median=None, inliers=np.zeros(len(pts), dtype=bool),
               threshold=None, spread=None, n_found=len(pts))
    if len(pts) < min_found:
        out["reason"] = f"only {len(pts)} frame(s) found a spot (need {min_found})"
        return out
    xy = pts[:, :2]
    median = np.median(xy, axis=0)
    d = np.linalg.norm(xy - median, axis=1)
    threshold = min(max(tol, mad_k * float(np.median(d))), cap)
    inliers = d <= threshold
    out.update(median=median, inliers=inliers, threshold=threshold)
    n_in = int(inliers.sum())
    if n_in < min_found or n_in < min_inlier_frac * len(pts):
        out["reason"] = (f"the frames disagree: only {n_in} of {len(pts)} spots are within "
                         f"{threshold * 100:.1f} cm of the median (need {min_inlier_frac:.0%}, at "
                         f"least {min_found}) -- maybe two different open areas")
        return out
    spot = pts[inliers].mean(axis=0)
    spread = float(np.sqrt(np.mean(np.sum((xy[inliers] - spot[:2]) ** 2, axis=1))))
    out.update(ok=True, spot=spot, spread=spread)
    return out


def grid_clearance(point, placement):
    """Distance (m) from point (camera frame) to the nearest blocked cell of the free-space grid
    in one frame's placement (table_detect's find_placement result); 0 if it isn't on a free cell."""
    free = placement["free"]
    ex, ey = placement["axes"]
    a0, b0 = placement["grid_origin"]
    cell = placement["cell_size"]
    rel = np.asarray(point, dtype=float) - placement["origin"]
    ia = int(np.floor((rel @ ex - a0) / cell))
    ib = int(np.floor((rel @ ey - b0) / cell))
    if not (0 <= ia < free.shape[0] and 0 <= ib < free.shape[1]) or not free[ia, ib]:
        return 0.0
    if "_clear" not in placement:   # cache: the same frame is asked about once per spot
        placement["_clear"] = ndimage.distance_transform_edt(free, sampling=cell)
    return float(placement["_clear"][ia, ib])


# ---------------------------------------------------------------------------
# The confirm picture
# ---------------------------------------------------------------------------

BAR = 64                        # px, button bar under the picture
BUTTONS = ("confirm", "rescan", "cancel")
LABELS = {"confirm": "Confirm [Enter]", "rescan": "Rescan [R]", "cancel": "Cancel [Esc]"}
GREEN, GREY, RED, WHITE = (0, 220, 0), (170, 170, 170), (255, 40, 40), (255, 255, 255)


def button_rects(width, height):
    """(x0, y0, x1, y1) per button, in the bar below a width x height picture."""
    w = width // len(BUTTONS)
    return {name: (i * w + 8, height + 8, (i + 1) * w - 8, height + BAR - 8)
            for i, name in enumerate(BUTTONS)}


def _draw_x(img, center, axes, intrinsics, color, size=0.02, width=3):
    ex, ey = axes
    for d in (ex + ey, ex - ey):
        uv, ok = project_to_pixels(np.stack([center - size * d, center + size * d]), **intrinsics)
        _draw_polyline(img, uv, ok, color, width)


def confirm_image(base, intrinsics, axes, candidates, inliers, spot, radius, lines, can_confirm=True):
    """RGB picture for the confirm window.

    base        HxWx3 RGB, already showing the table (red) and objects (yellow)
    candidates  Nx3 camera-frame spots, one per frame that found one; inliers: bool per spot
    spot        final camera-frame spot (None if refused); radius: its circle (m)
    lines       text shown top-left
    """
    img = np.ascontiguousarray(base[..., :3].astype(np.uint8).copy())
    h, w = img.shape[:2]
    for c, keep in zip(np.atleast_2d(candidates), inliers):
        if keep:
            draw_placement(img, dict(point=c, radius=radius, axes=axes), intrinsics, color=GREY, width=1)
        else:
            _draw_x(img, c, axes, intrinsics, RED)
    if spot is not None:
        draw_placement(img, dict(point=spot, radius=radius, axes=axes), intrinsics, color=GREEN, width=4)
    img = np.concatenate([img, np.full((BAR, w, 3), 40, np.uint8)], axis=0)
    try:
        import cv2
    except ImportError:
        return img          # no text or labels, but the circles are there
    if lines:   # on a darkened band, so it reads on any picture
        widths = [cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0][0] for s in lines]
        x1, y1 = min(w, max(widths) + 24), min(h, 14 + 26 * len(lines))
        img[:y1, :x1] = (img[:y1, :x1] * 0.35).astype(np.uint8)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (12, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1, cv2.LINE_AA)
    for name, (x0, y0, x1, y1) in button_rects(w, h).items():
        live = can_confirm or name != "confirm"
        fill = {"confirm": (0, 130, 0), "rescan": (90, 90, 90), "cancel": (150, 30, 30)}[name]
        cv2.rectangle(img, (x0, y0), (x1, y1), fill if live else (60, 60, 60), -1)
        cv2.putText(img, LABELS[name], (x0 + 14, (y0 + y1) // 2 + 7), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, WHITE if live else GREY, 2, cv2.LINE_AA)
    return img


def has_display():
    if sys.platform.startswith("linux"):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    return True


def ask(img, can_confirm=True, window=True, title="bowl spot"):
    """'confirm', 'rescan' or 'cancel'. Window: click a button, or Enter / R / Esc.
    Without a display (e.g. over plain ssh) or with window=False: a terminal prompt."""
    if window and has_display():
        try:
            return _ask_window(img, can_confirm, title)
        except Exception as e:        # cv2 without GUI support, or the window can't open
            print(f"(no window: {e}; asking here instead)")
    options = "[c]onfirm / [r]escan / [q]uit" if can_confirm else "[r]escan / [q]uit"
    while True:
        try:
            answer = input(f"{options}? ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            return "cancel"
        if answer in ("c", "confirm") and can_confirm:
            return "confirm"
        if answer in ("r", "rescan"):
            return "rescan"
        if answer in ("q", "quit", "cancel"):
            return "cancel"


def _ask_window(img, can_confirm, title):
    import cv2
    h = img.shape[0] - BAR
    rects = button_rects(img.shape[1], h)
    choice = {}

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONUP:
            for name, (x0, y0, x1, y1) in rects.items():
                if x0 <= x <= x1 and y0 <= y <= y1 and (can_confirm or name != "confirm"):
                    choice["v"] = name

    cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(title, on_mouse)
    cv2.imshow(title, img[..., ::-1])      # RGB -> BGR
    try:
        while "v" not in choice:
            key = cv2.waitKey(50) & 0xFF
            if key in (13, 10) and can_confirm:
                choice["v"] = "confirm"
            elif key in (ord("r"), ord("R")):
                choice["v"] = "rescan"
            elif key in (27, ord("q")):
                choice["v"] = "cancel"
            elif cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                choice["v"] = "cancel"      # window closed
    finally:
        cv2.destroyWindow(title)
        cv2.waitKey(1)
    return choice["v"]
