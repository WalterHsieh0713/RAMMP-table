"""Offline checks for place_bowl.py (no arm, no camera).

    python3 test_place_bowl.py            # the tests
    python3 test_place_bowl.py --sweep    # where on the table the arm can put the bowl (for REACH_BAND)
    python3 test_place_bowl.py --sweep -0.028   # ... for a table at that height (level frame, m)
"""

import sys

import numpy as np
from scipy.spatial.transform import Rotation

import place_bowl as P

SCAN_POSE = np.array([-0.10844, -0.86146, -3.11281, -2.18398, 0.03910, -0.99620, 1.59118])
TABLE_Z = 0.135     # measured 2026-10-04, arm_base_link
MODEL = P.ArmModel()
MODEL.R_wb, MODEL.up_base = np.eye(3), P.UP.copy()   # tests assume an upright arm, whatever mount.json says


def test_hand_is_level_with_camera_on_the_right():
    for yaw in np.radians([0, 37, 90, 160, -120]):
        d = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        R = Rotation.from_quat(P.hand_orientation(d)).as_matrix()
        assert np.allclose(R[:, 2], d)                      # points along the heading
        assert np.allclose(R[:, 0], [0, 0, 1])              # bowl level
        right = np.cross(d, [0, 0, 1])                      # right of the hand, seen from behind
        assert np.allclose(R[:, 1], right)                  # camera side (+y) on the right
        assert P.bowl_tilt_deg(R) < 1e-6


def test_camera_is_on_the_tool_plus_y_side():
    """The orientation above relies on this: the hand-eye calibration puts the camera at +y."""
    assert P.load_calibration()[1, 3] > 0.04


def test_bowl_geometry_from_the_measurements():
    assert np.isclose(P.HOLD_REACH, 0.055 + 0.05955)            # bowl radius + tool->fingertip
    assert np.isclose(P.RETREAT, 2.5 * 1.25 * 0.0254)            # 2.5 x the lip
    assert P.RETREAT > P.LIP_WIDTH                               # fingertips clear the lip
    tip = MODEL.fk(np.zeros(7), "finger_tip")[:3, 3]
    tool = MODEL.fk(np.zeros(7))[:3, 3]
    assert np.isclose(np.linalg.norm(tip - tool), P.TIP_AHEAD_OF_TOOL, atol=1e-4)


def test_retreat_keeps_the_hand_exactly_as_reported():
    quat = (Rotation.from_quat(P.hand_orientation([1.0, 1.0, 0.0]))
            * Rotation.from_euler("x", 3, degrees=True)).as_quat()   # a few deg off the model
    ee = np.r_[[0.6, 0.6, 0.2], quat]
    (name, pos, q), = P.retreat_poses(ee)
    assert np.allclose(q, quat)                                       # no turn
    back = (np.array([0.6, 0.6, 0.2]) - pos)
    assert np.isclose(np.linalg.norm(back), P.RETREAT) and abs(back[2]) < 1e-9


def test_tilted_mount_makes_true_level_the_reference():
    """Arm base tilted 16 deg: a hand that is level in the ROOM must plan with ~0 adjust, and
    'straight down' must be the room's down, not the base's."""
    up_base = Rotation.from_euler("y", -16, degrees=True).apply([0, 0, 1])
    m = P.ArmModel()
    m.R_wb, m.up_base = P.mount_rotation(up_base), up_base
    assert np.allclose(m.R_wb @ up_base, [0, 0, 1])
    # a hand level in the room: build it in the level frame, solve IK there
    q = m.ik([0.65, 0.10, 0.30], P.hand_orientation([1.0, 0.15, 0.0]), SCAN_POSE)
    assert q is not None
    moves, (zs, qs), info = P.plan_place(m, q, here=True, max_drop=0.03)
    assert info["adjust_deg"] < 0.5
    top, bottom = m.fk(qs[0])[:3, 3], m.fk(qs[-1])[:3, 3]
    assert np.linalg.norm((top - bottom)[:2]) < P.IK_TOL                 # straight down in the room
    assert np.linalg.norm((m.fk(qs[0], base=True) - m.fk(qs[-1], base=True))[:2, 3]) > 0.005  # not in base
    # the arm-reported hand (base frame) backs off horizontally in the room
    Tb = m.fk(qs[-1], base=True)
    ee = np.r_[Tb[:3, 3], Rotation.from_matrix(Tb[:3, :3]).as_quat()]
    (_, back, _), = P.retreat_poses(ee, m.R_wb)
    d = m.R_wb @ (back - Tb[:3, 3])
    assert abs(d[2]) < 1e-9 and np.isclose(np.linalg.norm(d), P.RETREAT)


def test_tool_sits_behind_the_bowl():
    spot = np.array([0.5, 0.4, TABLE_Z])
    pos = P.tool_above_spot(spot, TABLE_Z, 0.15)
    d = P.approach_direction(spot)
    assert np.isclose(np.linalg.norm(pos[:2] - spot[:2]), P.HOLD_REACH)
    assert np.allclose((spot - pos)[:2] / P.HOLD_REACH, d[:2])
    assert np.isclose(pos[2], TABLE_Z + 0.15)


def test_corridor_sees_only_what_is_in_the_hands_path():
    spot, d = np.array([0.7, 0.0, 0.1]), np.array([1.0, 0.0, 0.0])
    in_path = [0.7 - P.HOLD_REACH - 0.05, 0.0, 0.15]   # under the hand
    beside = [0.7 - P.HOLD_REACH, 0.20, 0.15]          # 20 cm to the side
    beyond = [0.7 + 0.15, 0.0, 0.15]                   # on the far side of the bowl
    flat = [0.7 - P.HOLD_REACH, 0.0, 0.105]            # 5 mm high: table noise
    hit = P.corridor_obstacles(np.array([in_path, beside, beyond, flat]), spot, 0.1, d)
    assert len(hit) == 1 and np.allclose(hit[0], in_path)


def test_ik_round_trip_and_held_joint():
    q0 = np.array([0.1, 0.5, 3.0, -1.5, 0.2, -1.0, 1.0])
    T = MODEL.fk(q0)
    quat = Rotation.from_matrix(T[:3, :3]).as_quat()
    q = MODEL.ik(T[:3, 3] - [0, 0, 0.03], quat, q0, fixed={5: q0[5]})
    assert q is not None and q[5] == q0[5]
    assert np.allclose(MODEL.fk(q)[:3, 3], T[:3, 3] - [0, 0, 0.03], atol=P.IK_TOL)


def holding_pose(spot):
    """A plausible start: bowl held level near the spot, a little high, slightly off-square."""
    d = P.approach_direction(spot)
    pos = P.tool_above_spot(spot, TABLE_Z, 0.25) - 0.05 * d
    tweak = Rotation.from_euler("xz", [4, 5], degrees=True)    # 4 deg roll, 5 deg yaw off
    quat = (Rotation.from_quat(P.hand_orientation(d)) * tweak).as_quat()
    return MODEL.ik(pos, quat, SCAN_POSE)


def test_full_plan_keeps_the_bowl_level_and_goes_straight_down():
    spot = np.array([0.90, 0.10, TABLE_Z])
    q_start = holding_pose(spot)
    assert q_start is not None
    record = dict(spot=spot.tolist(), table_z=TABLE_Z)
    moves, (zs, qs), info = P.plan_place(MODEL, q_start, record)
    assert 4 < info["adjust_deg"] < 8
    assert "hover" not in dict(moves) and "down" not in dict(moves)          # no stops on the way down
    T_top = MODEL.fk(qs[0])                                                 # the one stop: above the spot
    assert np.allclose(T_top[:3, 3], MODEL.fk(moves[-1][1])[:3, 3], atol=P.IK_TOL)
    assert P.bowl_tilt_deg(T_top[:3, :3]) < 1.0
    assert np.isclose(T_top[2, 3], TABLE_Z + P.SAFE_HEIGHT, atol=P.IK_TOL)
    centre = T_top[:3, 3] + P.HOLD_REACH * T_top[:3, 2]                     # bowl centre over the spot
    assert np.linalg.norm(centre[:2] - spot[:2]) < P.IK_TOL
    assert zs[0] - zs[-1] <= P.MAX_DROP + 1e-9                               # one move, <= 40 cm
    T_hover = T_top
    for q in qs:                                                              # straight down, level
        T = MODEL.fk(q)
        assert np.linalg.norm(T[:2, 3] - T_hover[:2, 3]) < P.IK_TOL
        assert P.bowl_tilt_deg(T[:3, :3]) <= P.MAX_LOWER_TILT_DEG
    assert np.isclose(zs[-1], TABLE_Z + P.BOWL_DEPTH - P.PRESS_BELOW, atol=P.PATH_STEP)


def test_refuses_a_big_turn_with_the_bowl_in_hand():
    spot = np.array([0.90, 0.10, TABLE_Z])
    d = P.approach_direction(spot)
    pos = P.tool_above_spot(spot, TABLE_Z, 0.25)
    camera_on_top = (Rotation.from_quat(P.hand_orientation(d))
                     * Rotation.from_euler("z", 90, degrees=True)).as_quat()
    q = MODEL.ik(pos, camera_on_top, SCAN_POSE)
    try:
        P.plan_place(MODEL, q, dict(spot=spot.tolist(), table_z=TABLE_Z))
    except P.Refused as e:
        assert "deg from level" in str(e)
    else:
        raise AssertionError("a 90 deg turn with the bowl in hand was not refused")


def test_a_target_given_by_hand_plans_like_a_looked_one():
    args = P.build_parser().parse_args(["place", "--at", "0.85", "-0.1", "--table-z", str(TABLE_Z)])
    assert args.at == [0.85, -0.1] and args.table_z == TABLE_Z and not args.execute
    record = P.manual_spot(*args.at, args.table_z, P.load_mount()[1], "at")
    assert record["confirmed"] and record["spot"] == [0.85, -0.1, TABLE_Z]
    P.check_spot(record, max_age=P.MAX_AGE_MIN)     # fresh, confirmed, current level -> accepted
    P.plan_place(MODEL, holding_pose(np.array(record["spot"])), record)


def sweep():
    """Bowl-centre distances (from the arm base axis) where the whole place plans."""
    # One line per distance as soon as it's planned (the full plan takes a while on the Jetson).
    # ok = the whole place plans; -- = refused (reason shown); no-hold = no start pose found.
    for yaw in (0, 45, -45, 90):
        for r in np.arange(0.30, 1.25, 0.05):
            spot = np.array([r * np.cos(np.radians(yaw)), r * np.sin(np.radians(yaw)), TABLE_Z])
            q = holding_pose(spot)
            if q is None:
                result = "no-hold"
            else:
                try:
                    P.plan_place(MODEL, q, dict(spot=spot.tolist(), table_z=TABLE_Z))
                    result = "ok"
                except P.Refused as e:
                    result = f"-- {str(e)[:70]}"
            print(f"yaw {yaw:4d}  r {r:.2f}: {result}", flush=True)


if __name__ == "__main__":
    if "--sweep" in sys.argv:
        i = sys.argv.index("--sweep")
        if len(sys.argv) > i + 1:
            TABLE_Z = float(sys.argv[i + 1])    # holding_pose and sweep read the global
        print(f"table z={TABLE_Z:.3f} m, URDF {P.ROBOT_URDF}, flange->fingertip "
              f"{P.TIP_AHEAD_OF_TOOL:.3f} m past the tool frame")
        sweep()
        sys.exit()
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"\n{len(tests)}/{len(tests)} passed")
