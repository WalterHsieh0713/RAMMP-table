"""Offline tests for spot_vote.py (several scans -> one spot) -- no ROS, no arm, no pybullet.

    python3 test_spot_vote.py           (or: pytest -q test_spot_vote.py)
    python3 test_spot_vote.py --show    # also open the confirm window on a noisy synthetic scan
"""
import sys

import numpy as np

import spot_vote as V
from table_detect import BOWL_RADIUS, MARGIN, detect_table, overlay_image, project_to_pixels
from test_placement import Scene, check_circle_is_clear, three_box_scene


def ring(center, n, radius, rng):
    """n points scattered within `radius` of center (x, y), z = 0."""
    ang = rng.uniform(0, 2 * np.pi, n)
    r = radius * np.sqrt(rng.uniform(0, 1, n))
    return np.column_stack([center[0] + r * np.cos(ang), center[1] + r * np.sin(ang), np.zeros(n)])


def test_one_outlier_is_dropped_and_the_rest_averaged():
    rng = np.random.default_rng(1)
    good = ring((0.85, 0.10), 9, 0.01, rng)
    pts = np.vstack([good, [[0.95, 0.10, 0.0]]])     # one 10 cm away
    vote = V.combine_spots(pts)
    assert vote["ok"], vote["reason"]
    assert vote["inliers"].tolist() == [True] * 9 + [False]
    assert np.allclose(vote["spot"], good.mean(axis=0))
    assert vote["spread"] < 0.01


def test_two_equal_clusters_are_refused():
    rng = np.random.default_rng(2)
    pts = np.vstack([ring((0.80, 0.0), 5, 0.005, rng), ring((0.90, 0.0), 5, 0.005, rng)])
    vote = V.combine_spots(pts)
    assert not vote["ok"] and "disagree" in vote["reason"]


def test_a_clear_majority_cluster_wins():
    rng = np.random.default_rng(3)
    major = ring((0.80, 0.0), 7, 0.005, rng)
    pts = np.vstack([major, ring((0.90, 0.0), 3, 0.005, rng)])
    vote = V.combine_spots(pts)
    assert vote["ok"], vote["reason"]
    assert np.allclose(vote["spot"], major.mean(axis=0))


def test_too_few_spots_are_refused():
    for n in (0, 1, 2):
        vote = V.combine_spots(np.full((n, 3), 0.85))
        assert not vote["ok"] and "frame" in vote["reason"], n


def test_identical_spots_are_all_inliers():
    vote = V.combine_spots(np.tile([0.85, 0.1, 0.13], (10, 1)))
    assert vote["ok"] and vote["inliers"].all() and vote["spread"] < 1e-12


def test_projection_round_trip_and_circle_drawn_around_it():
    intr = dict(fx=615.0, fy=615.0, cx=319.5, cy=239.5)
    p = np.array([0.05, -0.03, 0.6])
    uv, ok = project_to_pixels(p, **intr)
    u, v = uv[0]
    assert ok[0]
    back = np.array([(u - intr["cx"]) / intr["fx"] * p[2], (v - intr["cy"]) / intr["fy"] * p[2], p[2]])
    assert np.allclose(back, p)
    img = np.zeros((480, 640, 3), np.uint8)
    V.draw_placement(img, dict(point=p, radius=0.05, axes=(np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))),
                     intr, color=(0, 255, 0), width=1, samples=360)
    vs, us = np.nonzero(img[..., 1])
    assert abs(us.mean() - u) < 1.0 and abs(vs.mean() - v) < 1.0   # circle (+ cross) centred on it


def test_grid_clearance_matches_the_detector_and_is_zero_on_an_object():
    scene = three_box_scene()
    pl = scene.detect()["placement"]
    clear = V.grid_clearance(pl["point"], pl)
    assert abs(clear - pl["clearance"]) <= pl["cell_size"] + 1e-9, (clear, pl["clearance"])
    on_box = scene.origin + (-0.13) * scene.ex + 0.05 * scene.ey      # centre of the near-left box
    assert V.grid_clearance(on_box, pl) == 0.0
    off_table = scene.origin + 0.5 * scene.ex                          # past the table edge
    assert V.grid_clearance(off_table, pl) == 0.0


def roomy_scene():
    """A bigger table with one box: a hand passing over the spot moves it, rather than leaving no room."""
    return Scene(boxes=[(-0.15, 0.30, 0.07, 0.07, 0.09)], a_range=(-0.3, 0.3), b_range=(-0.2, 0.5))


def noisy_scans(scene, n=10, noise=0.002, seed=0, bad=()):
    """n detections of the scene with depth noise; frames in `bad` see an extra 5 cm box at the
    spot the clean frames pick (a hand passing through), so their spot jumps elsewhere."""
    rng = np.random.default_rng(seed)
    clean = scene.depth()
    pick = scene.detect()["placement"]["point"]
    a, b, _ = scene.to_table(pick)[0]
    blocked = Scene(boxes=list(scene.boxes) + [(a, b, 0.05, 0.05, 0.10)],
                    a_range=scene.a_range, b_range=scene.b_range)
    results = []
    for i in range(n):
        depth = (blocked.depth() if i in bad else clean).copy()
        depth[depth > 0] += rng.normal(0, noise, int((depth > 0).sum()))
        results.append(detect_table(depth, scene.intrinsics, up=None))
    return results


def averaged(scene, results):
    """Combine the frames' spots in table coordinates (a stand-in for the level frame)."""
    cands = [r["placement"]["point"] for r in results if r and r["placement"] is not None]
    vote = V.combine_spots(scene.to_table(cands))
    if vote["ok"]:
        a, b, _ = vote["spot"]
        vote["spot_cam"] = scene.origin + a * scene.ex + b * scene.ey
    return cands, vote


def test_a_frame_with_something_passing_through_is_dropped():
    scene = roomy_scene()
    results = noisy_scans(scene, bad=(4,))
    cands, vote = averaged(scene, results)
    assert len(cands) == 10 and vote["ok"], vote["reason"]
    assert not vote["inliers"][4], "the frame with the passing object should be the outlier"
    ref = results[0]["placement"]
    check_circle_is_clear(scene, dict(point=vote["spot_cam"], radius=BOWL_RADIUS, axes=ref["axes"]), "averaged: ")
    assert V.grid_clearance(vote["spot_cam"], ref) >= BOWL_RADIUS + MARGIN
    print(f"  {vote['inliers'].sum()} of {vote['n_found']} agree, spread {vote['spread'] * 1000:.1f} mm")


def test_noisy_scans_of_a_cluttered_table_give_a_clear_spot():
    """With 2 mm depth noise some frames hop ~2 cm to an almost equally good spot; those are
    dropped, and the average of the rest must still be on bare table."""
    scene = three_box_scene()
    results = noisy_scans(scene)
    _, vote = averaged(scene, results)
    assert vote["ok"], vote["reason"]
    check_circle_is_clear(scene, dict(point=vote["spot_cam"], radius=BOWL_RADIUS,
                                      axes=results[0]["placement"]["axes"]), "averaged: ")
    print(f"  {vote['inliers'].sum()} of {vote['n_found']} agree, spread {vote['spread'] * 1000:.1f} mm")


def test_confirm_image_has_the_bar_and_the_circles():
    scene = three_box_scene()
    res = scene.detect()
    pl = res["placement"]
    base = overlay_image(np.full(scene.shape + (3,), 128, np.uint8), res["mask"], obstacle_mask=res["obstacle_mask"])
    cands = np.array([pl["point"], pl["point"] + 0.1 * pl["axes"][0]])
    img = V.confirm_image(base, scene.intrinsics, pl["axes"], cands, [True, False], pl["point"],
                          BOWL_RADIUS, ["line one", "line two"])
    assert img.shape == (scene.shape[0] + V.BAR, scene.shape[1], 3)
    assert (img[: scene.shape[0]] == V.GREEN).all(axis=2).sum() > 100     # the final circle
    assert (img[: scene.shape[0]] == V.RED).all(axis=2).sum() > 10        # the outlier's X


def test_terminal_prompt():
    answers = iter(["", "x", "c"])
    V.input = lambda _prompt: next(answers)     # spot_vote looks up input() in its own module first
    try:
        assert V.ask(None, window=False) == "confirm"
        answers = iter(["c", "r"])               # confirm not offered when refused
        assert V.ask(None, can_confirm=False, window=False) == "rescan"
        answers = iter(["q"])
        assert V.ask(None, window=False) == "cancel"
    finally:
        del V.input


def show():
    scene = roomy_scene()
    results = noisy_scans(scene, bad=(4,))
    cands, vote = averaged(scene, results)
    res = results[0]
    base = overlay_image(np.full(scene.shape + (3,), 128, np.uint8), res["mask"], obstacle_mask=res["obstacle_mask"])
    img = V.confirm_image(base, scene.intrinsics, res["placement"]["axes"], cands, vote["inliers"], vote["spot_cam"],
                          BOWL_RADIUS, [f"{vote['inliers'].sum()} of {len(cands)} agree, spread "
                                        f"{vote['spread'] * 100:.1f} cm (synthetic)"])
    print("you chose:", V.ask(img))


def main():
    if "--show" in sys.argv:
        return show()
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
