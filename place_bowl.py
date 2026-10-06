"""Put the held bowl down on a clear spot of the table, sensing the touch-down.

Two separate steps (the held bowl would block the wrist camera, so the table is looked at first):

    python3 place_bowl.py level                 # once per mounting: hand held truly level -> true "up"
    python3 place_bowl.py look                  # gripper empty, arm at the scan pose: find the spot,
                                                # click Confirm (or Rescan / Cancel) -> saved
    (grip the bowl by its lip: hand level, camera on the right seen from behind)
    python3 place_bowl.py place                 # dry run: plan + check everything, nothing moves
    python3 place_bowl.py place --execute       # do it ([ENTER] before moving and before letting go)
    python3 place_bowl.py place --here --execute   # skip the spot: lower straight down from where it is

A target given by hand instead of `look` (no camera; you keep the target clear):
    python3 place_bowl.py mark                  # fingertip touching the target -> saved as the spot
    python3 place_bowl.py place --at X Y [--table-z Z] [--execute]   # bowl centre at X, Y (m)
  X, Y, Z are in the level frame: origin on the arm base axis, x/y the arm base's (x forward, y left,
  as the Kinova base is marked) turned only by the mount tilt, z true up. `mark` prints them.

From Python:  spot = choose_spot();  if spot: place_at(spot, execute=True)

look
  --frames (10) depth frames from the wrist camera, --period (0.2 s) apart. The camera pose comes
  from the arm's joint angles (forward kinematics on the repo URDF) + the saved hand-eye
  calibration, so no TF is needed. In each frame table_detect finds the table (floor rejected by
  height and distance) and a clear spot the arm can reach. spot_vote drops the outlying spots and
  averages the rest, then checks the average is still clear (two clusters can average onto an
  object). A window shows every frame's spot and the result: Confirm saves the spot, the table
  height and the obstacle points to ~/.table_place/ (level frame), with confirmed: true; `place`
  refuses a spot without it (--no-confirm overrides). No display (plain ssh): --no-window, or it
  asks in the terminal and the picture is in ~/.table_place/overlay.png. Nothing moves.

place  (starts with the bowl already gripped by its lip)
  1. adjust   : turn the hand in place to EXACTLY level + rolled 90 deg (camera on the right of
                the hand, seen from behind the arm; tool z = forward, tool x = up). Only a small
                correction is allowed (<= 15 deg): a bigger turn would tip the bowl.
  2. lift     : straight up to 0.30 m above the table (if lower).
  3. transit  : at that height, to above the spot (hand pointing from the arm base at the spot).
  4. down     : from there, ONE continuous move straight down (2 cm/s) until the torques say
                the bowl is on the table -- at most 40 cm, and never more than 3 cm below where
                the table should be. Then, without a prompt: open the gripper and pull out.
     Steps 1-4 are joint moves to IK solutions (position control, "low" speed). Where a joint
     move would tilt the bowl > 10 deg or dip below the table, it is split with extra waypoints.
  (old, still used by --impedance:)
  5. down     : straight down to 3 cm above the table (one joint move, checked to stay on the
                straight line and keep the bowl level).
  6. touch    : the rest in ONE slow joint trajectory (2 cm/s, position control) while the
                joint torques are read on a second connection ~50x/s; when they jump (the bowl's
                weight comes off the arm), stop_action, then confirm standing still (a false
                alarm carries on down). --steps: instead 2 mm joint steps. After each step the arm
                settles and the joint torques are read; when the bowl touches, part of its
                weight comes off the arm and the torques jump -> stop. At most one 2 mm step
                past the first touch, and never lower than 1 cm below where the table should be.
     --impedance: instead, the last 3 cm under the lab's joint compliant mode -- REFUSED while
                the arm base is tilted (it is, ~16 deg): the controller's gravity model assumes an
                upright base. Its gravity
                model has J6 fixed at -67.6 deg (with J6 elsewhere the wrist falls -- it slammed
                into the table on 2026-10-04), so every pose from the transit down is planned
                with J6 = -67.6. In this setup that means the flipped-elbow posture (J4 ~ +90):
                the arm must already be in it, since switching mid-place is a ~175 deg move.
  7. open_gripper() (no prompt) (fingers pinch the bowl's lip).
  8. retreat  : straight back along the hand, 2.5 x the lip width, so the fingers slide off the
                lip without touching it. Then stop.

Where the bowl is, relative to the hand (all along the hand's pointing direction):
  tool frame --5.96 cm--> fingertip --BOWL_RADIUS (5.5 cm)--> bowl centre
  The fingertips are pushed in to the bowl wall: the lip (LIP_WIDTH = 1.25 in) sticks out past
  BOWL_RADIUS, so its inner edge is at the wall. Fingertip offset: URDF finger_tip (17.955 cm from
  the flange) - tool_frame (12 cm) = scene_description.tool_frame_to_finger_tip (0.05955).
  The fingers pinch the lip, so the lip is at the tool frame's height and the bowl bottom is
  BOWL_DEPTH (8 cm) below it. The spot search keeps clear room for the whole lip
  (BOWL_RADIUS + LIP_WIDTH = 8.7 cm).
  Everything is planned and checked before anything moves; each move must arrive within 1 deg,
  and the bowl must still be level and the hand where the plan says.
"""

import argparse
import json
import os
import sys
import textwrap
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from arm_backend import connect_arm  # (which arm stack: TABLE_ARM_BACKEND)
from spot_vote import ask, combine_spots, confirm_image, grid_clearance
from table_detect import (BOWL_RADIUS, EDGE_MARGIN, MARGIN, MAX_PLANE_DIST, MIN_TABLE_HEIGHT,
                          backproject, depth_to_color, detect_table, overlay_image, plane_axes,
                          write_image)

HERE = Path(__file__).resolve().parent
# Per-machine paths: the defaults fit rchi-cpu-5; on another machine (e.g. the Jetson) export
# TABLE_URDF / TABLE_CALIB / TABLE_STATE_DIR / TABLE_CAMERA_NS instead of editing them here.
# The repo URDF: next to this folder when it lives inside feeding-deployment, else the lab checkout.
_URDF = Path("src") / "feeding_deployment" / "assets" / "robot" / "robot.urdf"
ROBOT_URDF = Path(os.environ["TABLE_URDF"]) if "TABLE_URDF" in os.environ else next(
    (p for p in (HERE.parent / _URDF, Path.home() / "feeding-deployment" / _URDF) if p.exists()),
    HERE.parent / _URDF)
CALIB_FILE = Path(os.environ.get(
    "TABLE_CALIB", Path.home() / ".ros2" / "easy_handeye2" / "calibrations" / "wrist_camera_calib.calib"))
CAMERA_NS = os.environ.get("TABLE_CAMERA_NS", "/camera/wrist")
SPOT_DIR = Path(os.environ.get("TABLE_STATE_DIR", Path.home() / ".table_place"))
SPOT_FILE = SPOT_DIR / "spot.json"
OBSTACLE_FILE = SPOT_DIR / "obstacles.npy"
MOUNT_FILE = SPOT_DIR / "mount.json"   # true "up" in the arm base frame (`place_bowl.py level`)

# --- the bowl and how it is held (see the diagram above) ---
LIP_WIDTH = 0.03175     # m, 1.25 in: the flat lip around the bowl, past BOWL_RADIUS
BOWL_DEPTH = 0.08       # m, lip (= tool frame height) to the bowl's bottom
TIP_AHEAD_OF_TOOL = 0.05955   # m, fingertip in front of the tool frame (URDF; scene_description)
HOLD_REACH = BOWL_RADIUS + TIP_AHEAD_OF_TOOL   # m, bowl centre in front of the tool frame
# --- the motion ---
SAFE_HEIGHT = 0.30      # m, tool frame above the table while carrying the bowl over to the spot
HOVER_GAP = 0.10        # m, bowl bottom this far above the table before going straight down
PRE_GAP = 0.03          # m, ... one move takes it down to this, then 2 mm steps with touch sensing
PRESS_BELOW = 0.03      # m, never lower than this below where the table should be
MIN_TOOL_HEIGHT = 0.02  # m, the tool frame is never targeted lower than this above the table
MAX_DROP = 0.40         # m, --here: keep going down until the table is felt -- at most this far
                        # (or as far as the arm can reach straight down, whichever is less)
RETREAT = 2.5 * LIP_WIDTH   # m, straight back after letting go, clear of the lip, then stop
MAX_ADJUST_DEG = 15.0   # the adjust step may only correct this much
MAX_BOWL_TILT_DEG = 10.0    # bowl tilt allowed anywhere along a joint move
MAX_LOWER_TILT_DEG = 1.0    # ... and on the straight way down
# --- touch-down ---
PATH_STEP = 0.002       # m, one joint step while feeling for the table
SETTLE_S = 0.4          # s, wait after each step before reading torques
FIRST_SETTLE_S = 1.0    # s, wait after the last positioning move before the baseline
TORQUE_SAMPLES = 10     # readings averaged per step (20 for the baseline)
NOISE_FACTOR = 5.0      # contact also needs a change >= this x the measured torque noise
CONFIRM_S = 0.3         # an over-threshold reading must still be over after this long
# continuous lowering (default; --steps for the step-by-step version above)
SLIDE_SPEED = 0.02      # m/s: waypoints SLIDE_SPEED * 0.5 s apart (kinova.py gives each 0.5 s)
POLL_S = 0.02           # s between torque readings while moving
WINDOW = 5              # readings averaged while moving
BASELINE_S = 0.4        # s of motion averaged as the moving baseline ...
BASELINE_LAG_S = 0.3    # ... ending this long ago: a ROLLING reference. The gravity load drifts
                        # as the arm goes down (it passed 1.5 Nm after ~10 cm on 2026-10-05 and
                        # caused a mid-air stop); a contact changes the torque within ~0.1 s.
MOVE_TORQUE = 1.5       # Nm: contact while moving = a change over this (and NOISE_FACTOR x noise)
HITS = 3                # ... in this many readings in a row -> stop_action
MAX_RESUMES = 2         # false alarms tolerated before giving up
TOUCH_JOINTS = [1, 3, 5]    # J2, J4, J6: the joints that carry the bowl's weight
STEP_TORQUE = 1.0       # Nm, a change this big in one step on a TOUCH_JOINT = contact ...
TOTAL_TORQUE = 2.0      # Nm, ... or this big since the start of the steps

# --- --impedance: the lab's joint compliant mode for the last few cm ---
# Its gravity model (urdfs/hack_gen3_robotiq_2f_85.urdf) has J6 FIXED at this angle -- checked:
# that model's tool pose equals ours at J6 = -67.6 deg to 0.0 mm, and is 53 cm off at +67.6.
# With J6 anywhere else it compensates gravity for the wrong wrist and the wrist falls (it
# slammed into the table on 2026-10-04 with J6 at +60). So with --impedance the arm is put
# at J6 = HACK_J6 before switching, and the script refuses to switch otherwise.
HACK_J6 = -1.18039928
HACK_J6_TOL_DEG = 2.0
COMPLIANT_STEP = 0.005  # m, IK every 5 mm down the compliant descent
STREAM_HZ = 10.0        # the controller holds still if commands stop for 0.25 s
LOWER_SPEED = 0.02      # m/s
LEAD = 0.015            # m, the target is never more than this below the hand
STALL_WINDOW_S = 0.5    # contact: the hand moved less than STALL_DIST down in this long ...
STALL_DIST = 0.001      # m
PRESS = 0.005           # m, after contact, target this far below the hand (light press)
HOLD_S = 0.5            # s, hold the press before switching back
MAX_SAG = 0.02          # m, hand drop allowed when compliant mode takes over
# The controller's own soft joint limits (compliant_controller.py: limits minus 15 deg).
COMPLIANT_LIMITS = {1: 2.24 - np.radians(15), 3: 2.57 - np.radians(15)}   # J2, J4

# --- look ---
LOOK_FRAMES = 10        # frames scanned and averaged (>= 5 for the outlier vote to mean much)
LOOK_PERIOD = 0.2       # s between them
SELF_RADIUS = 0.10      # m, depth points this close to our own gripper are the gripper
HAND_HALF_WIDTH = 0.07  # m, half-width of the strip the hand + wrist come down through
HAND_BEHIND = 0.20      # m, the hand + wrist reach this far behind the tool frame
# Where the bowl centre can go: distance from the arm base axis. The whole place plans in this
# ring in every direction swept (table 0.135 m above the arm base). Redo the sweep
# (test_place_bowl.py --sweep) if the table height, the hold or the bowl changes.
REACH_BAND = (0.75, 0.95)
MAX_TABLE_TILT_DEG = 10.0

# --- checks ---
IK_TOL = 0.005               # m
IK_ROT_TOL_DEG = 1.0
MAX_STEP_JUMP_DEG = 120.0    # one joint move
ARRIVE_TOL_DEG = 1.0
TOOL_FRAME_TOL = 0.015       # m, arm-reported tool pose vs URDF tool_frame
MAX_AGE_MIN = 30.0           # a saved spot older than this is refused (the base may have moved)
REQUIRED_SPEED = "low"
ARM_JOINTS = [f"joint_{i}" for i in range(1, 8)]
CONTINUOUS = [0, 2, 4, 6]
UP = np.array([0.0, 0.0, 1.0])


# ---------------------------------------------------------------------------
# Geometry (pure numpy -- tested offline in test_place_bowl.py)
# ---------------------------------------------------------------------------

def hand_orientation(heading):
    """Level hand pointing along `heading`, rolled so the camera is on its right.

    tool z = heading (horizontal), tool y = the camera's side = right of the hand seen from
    behind = heading x up, tool x = up. The camera sits on the tool's +y side
    (wrist_camera_calib: +6.3 cm along y), so with tool y = up it would be on top.
    """
    z = np.array([heading[0], heading[1], 0.0], dtype=float)
    z /= np.linalg.norm(z)
    y = np.cross(z, UP)
    x = np.cross(y, z)
    return Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()


def bowl_tilt_deg(R):
    """How far the bowl is from level: tool x should point straight up."""
    return float(np.degrees(np.arccos(np.clip(R[:, 0] @ UP, -1.0, 1.0))))


def approach_direction(spot):
    """Horizontal unit vector from the arm base axis toward the spot."""
    d = np.array([spot[0], spot[1], 0.0], dtype=float)
    return d / np.linalg.norm(d)


def tool_above_spot(spot, table_z, height, reach=HOLD_REACH):
    """Tool position that puts the bowl centre over `spot`, tool `height` above the table."""
    d = approach_direction(spot)
    pos = np.array([spot[0], spot[1], table_z]) - reach * d
    pos[2] = table_z + height
    return pos


def corridor_obstacles(points, spot, table_z, approach, reach=HOLD_REACH):
    """Obstacle points (arm_base_link) inside the strip the hand + wrist come down through
    and back out of: from the tool's front at the bowl to the hand's rear after the retreat,
    HAND_HALF_WIDTH either side, from just above the table to the hover height."""
    if len(points) == 0:
        return points
    rel = points - np.array([spot[0], spot[1], table_z])
    along = rel @ approach                       # + toward the spot centre
    side = rel @ np.array([-approach[1], approach[0], 0.0])
    near = -(reach + RETREAT + HAND_BEHIND)
    far = -BOWL_RADIUS + 0.01                    # the fingertips
    inside = ((along > near) & (along < far) & (np.abs(side) < HAND_HALF_WIDTH)
              & (rel[:, 2] > 0.02) & (rel[:, 2] < BOWL_DEPTH + HOVER_GAP + 0.05))
    return points[inside]


def wrap(rad):
    return (np.asarray(rad) + np.pi) % (2 * np.pi) - np.pi


def wrap_deg(rad):
    """Angle difference in radians -> degrees in [-180, 180)."""
    return np.degrees(wrap(rad))


def pose_matrix(pos, quat):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat).as_matrix()
    T[:3, 3] = pos
    return T


def load_calibration(path=CALIB_FILE):
    """end_effector_link -> camera_color_optical_frame, from the easy_handeye2 file."""
    import yaml
    calib = yaml.safe_load(Path(path).read_text())["transform"]
    t, q = calib["translation"], calib["rotation"]
    return pose_matrix([t["x"], t["y"], t["z"]], [q["x"], q["y"], q["z"], q["w"]])


def load_mount(path=MOUNT_FILE):
    """(R_wb, up_base): rotation taking arm-base vectors into a frame whose z is TRUE up, and
    true up in base coordinates. The arm on this rig is mounted ~16 deg tilted (2026-10-04),
    so base z is not up. Identity until `place_bowl.py level` has been run."""
    try:
        up = np.asarray(json.loads(Path(path).read_text())["up_base"], dtype=float)
    except (FileNotFoundError, KeyError, ValueError):
        return np.eye(3), UP.copy()
    up /= np.linalg.norm(up)
    return mount_rotation(up), up


def mount_rotation(up_base):
    """Smallest rotation taking up_base onto +z (no extra spin about the vertical)."""
    up_base = np.asarray(up_base, dtype=float) / np.linalg.norm(up_base)
    axis = np.cross(up_base, UP)
    angle = np.arccos(np.clip(up_base @ UP, -1.0, 1.0))
    if np.linalg.norm(axis) < 1e-9:
        return np.eye(3)
    return Rotation.from_rotvec(axis / np.linalg.norm(axis) * angle).as_matrix()


# ---------------------------------------------------------------------------
# Arm model (PyBullet FK on the repo URDF, least-squares IK)
# ---------------------------------------------------------------------------

class ArmModel:
    def __init__(self, urdf=ROBOT_URDF):
        import pybullet as p
        self.p = p
        self.cid = p.connect(p.DIRECT)
        self.robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=self.cid)
        joints, links = {}, {}
        for i in range(p.getNumJoints(self.robot, physicsClientId=self.cid)):
            info = p.getJointInfo(self.robot, i, physicsClientId=self.cid)
            joints[info[1].decode()] = info
            links[info[12].decode()] = i
        self.arm = [joints[name][0] for name in ARM_JOINTS]
        # (lower, upper) in rad; continuous joints come back as lower > upper.
        self.limits = [(joints[name][8], joints[name][9]) for name in ARM_JOINTS]
        self.links = links
        # Everything the planner sees is in the LEVEL frame: the arm base frame turned so z is
        # true up (same origin). The arm itself takes base-frame commands; joint commands don't
        # care, and the few Cartesian ones convert with R_wb.
        self.R_wb, self.up_base = load_mount()

    def fk(self, q, link="tool_frame", base=False):
        """Pose of `link` in the level frame (base=True: in the arm's own base frame)."""
        for j, v in zip(self.arm, q):
            self.p.resetJointState(self.robot, j, float(v), physicsClientId=self.cid)
        s = self.p.getLinkState(self.robot, self.links[link], computeForwardKinematics=True,
                                physicsClientId=self.cid)
        T = pose_matrix(s[4], s[5])
        if not base:
            T[:3, :] = self.R_wb @ T[:3, :]
        return T

    def limit_violations(self, q):
        bad = []
        for i, (v, (lo, hi)) in enumerate(zip(q, self.limits)):
            if lo < hi and not (lo <= v <= hi):
                bad.append(f"J{i + 1} {np.degrees(v):.0f} deg (limit {np.degrees(lo):.0f}..{np.degrees(hi):.0f})")
        return bad

    def ik(self, pos, quat, seed, fixed=None, restarts=10, rot_weight=0.2, rot_tol=IK_ROT_TOL_DEG,
           stay=0.0):
        """Joint angles putting tool_frame at (pos, quat), or None.

        fixed = {joint index: angle} holds those joints. rot_weight is metres per radian of
        orientation error (lower = position first); rot_tol in deg. stay > 0 pulls the answer
        toward the seed (m per rad): the arm has 7 joints for a 6-number pose, so without it
        consecutive small steps can slide along the spare freedom. Tries the seed first, then random
        starts; of the solutions within IK_TOL / IK_ROT_TOL_DEG and the joint limits, returns
        the one that moves the joints least from the seed. Continuous joints come back in
        [-pi, pi).
        """
        fixed = fixed or {}
        seed = np.asarray(seed, dtype=float)
        free = [i for i in range(7) if i not in fixed]
        target_R = Rotation.from_quat(quat)
        pos = np.asarray(pos, dtype=float)

        def full(x):
            q = seed.copy()
            q[free] = x
            for i, v in fixed.items():
                q[i] = v
            return q

        def residual(x):
            T = self.fk(full(x))
            rot = (Rotation.from_matrix(T[:3, :3]) * target_R.inv()).as_rotvec()
            return np.concatenate([T[:3, 3] - pos, rot_weight * rot, stay * wrap(x - seed[free])])

        lo = np.array([self.limits[i][0] if self.limits[i][0] < self.limits[i][1] else -2 * np.pi for i in free])
        hi = np.array([self.limits[i][1] if self.limits[i][0] < self.limits[i][1] else 2 * np.pi for i in free])
        rng = np.random.default_rng(0)
        best = None
        for k in range(restarts + 1):
            x0 = (np.clip(seed[free], lo + 1e-6, hi - 1e-6) if k == 0
                  else rng.uniform(np.maximum(lo, -np.pi), np.minimum(hi, np.pi)))
            # PyBullet FK is single precision: the default finite-difference step is too small.
            r = least_squares(residual, x0, bounds=(lo, hi), diff_step=1e-4,
                              xtol=1e-12, ftol=1e-12, gtol=1e-12)
            q = full(r.x)
            q[CONTINUOUS] = wrap(q[CONTINUOUS])
            T = self.fk(q)
            err = np.linalg.norm(T[:3, 3] - pos)
            rot = np.degrees((Rotation.from_matrix(T[:3, :3]) * target_R.inv()).magnitude())
            if err > IK_TOL or rot > rot_tol or self.limit_violations(q):
                continue
            move = float(np.max(np.abs(wrap_deg(q - seed))))
            if best is None or move < best[0]:
                best = (move, q)
            if k == 0 and move < 30.0:
                break           # the seed's own solution is close: good enough
        return None if best is None else best[1]


# ---------------------------------------------------------------------------
# Planning (no arm needed)
# ---------------------------------------------------------------------------

class Refused(Exception):
    pass


def check_joint_move(model, q_a, q_b, table_z, samples=20):
    """Problems along a straight joint-space move (what Kortex reach_joint_angles does)."""
    problems = []
    delta = wrap(q_b - q_a)
    # The adjust step may start from a bowl tilted up to MAX_ADJUST_DEG; never worse than the ends.
    allowed = max(MAX_BOWL_TILT_DEG, bowl_tilt_deg(model.fk(q_a)[:3, :3]) + 1.0,
                  bowl_tilt_deg(model.fk(q_b)[:3, :3]) + 1.0)
    for s in np.linspace(0.0, 1.0, samples):
        T = model.fk(q_a + s * delta)
        if bowl_tilt_deg(T[:3, :3]) > allowed:
            problems.append(f"bowl tilts {bowl_tilt_deg(T[:3, :3]):.0f} deg")
            break
        if table_z is not None and T[2, 3] < table_z + MIN_TOOL_HEIGHT:
            problems.append(f"hand dips to {(T[2, 3] - table_z) * 100:.1f} cm above the table")
            break
    return problems


def plan_segment(model, q_from, pos_to, quat_to, table_z, name, depth=0):
    """Joint waypoints from q_from to the pose, split in Cartesian halves until each joint
    move keeps the bowl level and above the table. Returns [(name, q), ...]."""
    q_to = model.ik(pos_to, quat_to, q_from)
    if q_to is None:
        raise Refused(f"{name}: no IK solution for tool at {np.round(pos_to, 3).tolist()}")
    jump = float(np.max(np.abs(wrap_deg(q_to - q_from))))
    problems = check_joint_move(model, q_from, q_to, table_z)
    if jump <= MAX_STEP_JUMP_DEG and not problems:
        return [(name, q_to)]
    if depth >= 4:
        raise Refused(f"{name}: " + ("; ".join(problems) or f"a joint moves {jump:.0f} deg"))
    T_from = model.fk(q_from)
    mid_pos = (T_from[:3, 3] + np.asarray(pos_to)) / 2
    rots = Rotation.from_matrix(np.stack([T_from[:3, :3], Rotation.from_quat(quat_to).as_matrix()]))
    mid_quat = Slerp([0, 1], rots)([0.5])[0].as_quat()
    first = plan_segment(model, q_from, mid_pos, mid_quat, table_z, name + "'", depth + 1)
    return first + plan_segment(model, first[-1][1], pos_to, quat_to, table_z, name, depth + 1)


def plan_descent(model, q_start, floor_z, step=PATH_STEP, impedance=False):
    """Joint targets every `step` straight down from q_start's tool pose to floor_z, bowl level.
    impedance=True holds J6 at HACK_J6 and keeps J2/J4 inside the compliant controller's limits.
    Returns (z values, joint arrays); the first entry is q_start itself."""
    fixed = {5: HACK_J6} if impedance else None
    T = model.fk(q_start)
    quat = Rotation.from_matrix(T[:3, :3]).as_quat()
    zs = np.arange(T[2, 3], floor_z - 1e-9, -step)
    qs, seed = [np.asarray(q_start, dtype=float)], q_start
    for z in zs[1:]:
        q = model.ik([T[0, 3], T[1, 3], z], quat, seed, restarts=0, stay=0.01, fixed=fixed)
        if q is None or np.max(np.abs(wrap_deg(q - seed))) > 3.0 * step / PATH_STEP:
            # The arm can't go further straight down from here: stop the plan at the last good
            # point (needs at least 1 cm of travel to be worth it).
            if T[2, 3] - zs[len(qs) - 1] >= 0.01:
                print(f"note: the arm can only go {(T[2, 3] - zs[len(qs) - 1]) * 100:.0f} cm straight down "
                      "from here (reach / joint limits)")
                zs = zs[:len(qs)]
                break
            raise Refused(f"lower: can't go straight down from z={T[2, 3]:.3f}"
                          + (" with J6 at -67.6" if impedance else ""))
        if impedance:
            for j, lim in COMPLIANT_LIMITS.items():
                if abs(q[j]) > lim:
                    raise Refused(f"lower: J{j + 1} {np.degrees(q[j]):.0f} deg at z={z:.3f} is past the "
                                  f"compliant controller's soft limit {np.degrees(lim):.0f}")
        qs.append(q)
        seed = q
    return zs, np.array(qs)


def check_straight(model, q_a, q_b, samples=20, max_dev=0.005):
    """Problems if the joint move q_a -> q_b strays from the straight line between its end
    poses by more than max_dev, or tilts the bowl more than MAX_LOWER_TILT_DEG."""
    a, b = model.fk(q_a)[:3, 3], model.fk(q_b)[:3, 3]
    ab = b - a
    if ab @ ab < 1e-8:          # no move to speak of
        return []
    delta = wrap(q_b - q_a)
    for s in np.linspace(0.0, 1.0, samples):
        T = model.fk(q_a + s * delta)
        p = T[:3, 3] - a
        dev = np.linalg.norm(p - (p @ ab) / (ab @ ab) * ab)
        if dev > max_dev:
            return [f"leaves the straight line by {dev * 100:.1f} cm"]
        if bowl_tilt_deg(T[:3, :3]) > MAX_LOWER_TILT_DEG:
            return [f"tilts the bowl {bowl_tilt_deg(T[:3, :3]):.0f} deg"]
    return []


def plan_final(model, q_from, quat, pre_pos, floor_z, table_z, hover_pos=None, transit_pos=None):
    """transit -> hover -> straight down to the pre-contact pose, then the touch-down steps.
    Returns (moves [(name, q)], descent)."""
    moves, q = [], q_from
    for name, pos in (("transit", transit_pos), ("hover", hover_pos)):
        if pos is not None:
            moves += plan_segment(model, q, pos, quat, table_z, name)
            q = moves[-1][1]
    q_pre = model.ik(pre_pos, quat, q, restarts=0, stay=0.01)
    if q_pre is None:
        raise Refused(f"down: no IK solution near the hover pose for {np.round(pre_pos, 3).tolist()}")
    problems = check_straight(model, q, q_pre)
    if problems:
        raise Refused("down: " + "; ".join(problems))
    if np.max(np.abs(wrap_deg(q_pre - q))) > ARRIVE_TOL_DEG:
        moves.append(("down", q_pre))
    return moves, plan_descent(model, q_pre, floor_z)


def check_in_place(model, q_a, q_b, samples=20, max_wander=0.03):
    """Problems for a reconfiguring joint move that should leave the hand where it is: the
    tool may wander max_wander, never drop more than 1 cm, and the bowl tilt stays bounded."""
    T_a = model.fk(q_a)
    allowed = max(MAX_BOWL_TILT_DEG, bowl_tilt_deg(T_a[:3, :3]) + 1.0)
    delta = wrap(q_b - q_a)
    for s in np.linspace(0.0, 1.0, samples):
        T = model.fk(q_a + s * delta)
        if np.linalg.norm(T[:3, 3] - T_a[:3, 3]) > max_wander:
            return [f"the hand wanders {np.linalg.norm(T[:3, 3] - T_a[:3, 3]) * 100:.0f} cm"]
        if T[2, 3] < T_a[2, 3] - 0.01:
            return [f"the hand dips {(T_a[2, 3] - T[2, 3]) * 100:.0f} cm"]
        if bowl_tilt_deg(T[:3, :3]) > allowed:
            return [f"the bowl tilts {bowl_tilt_deg(T[:3, :3]):.0f} deg"]
    return []


def plan_final_impedance(model, q_from, quat, pre_pos, floor_z, table_z, hover_pos=None,
                         transit_pos=None, samples=80):
    """Like plan_final, but every pose from the transit down has J6 at HACK_J6 (or within a few
    deg of it above the pre-contact pose), so the lab's compliant controller models the wrist
    right. Few arm configurations allow that AND a level straight descent with J6 held, so:
    sample configurations at the pre-contact pose with J6 = HACK_J6, keep those whose compliant
    descent works, and build hover/transit upward from them. Returns (moves, descent)."""
    rng = np.random.default_rng(0)
    seeds = [np.asarray(q_from, dtype=float)]
    for _ in range(samples):
        sd = rng.uniform(-np.pi, np.pi, 7)
        for i, (lo, hi) in enumerate(model.limits):
            if lo < hi:
                sd[i] = rng.uniform(lo, hi)
        seeds.append(sd)
    pres = []
    for sd in seeds:
        sd = sd.copy()
        sd[5] = HACK_J6
        q = model.ik(pre_pos, quat, sd, restarts=0, fixed={5: HACK_J6})
        if q is not None and not any(np.max(np.abs(wrap_deg(q - h))) < 2.0 for h in pres):
            pres.append(q)
    pres.sort(key=lambda q: float(np.max(np.abs(wrap_deg(q - q_from)))))
    reasons = []
    for q_pre in pres:
        try:
            descent = plan_descent(model, q_pre, floor_z, step=COMPLIANT_STEP, impedance=True)
        except Refused as e:
            reasons.append(str(e))
            continue
        moves, top = [("down", q_pre)], q_pre
        ok = True
        for name, pos, check in (("hover", hover_pos, lambda a, b: check_straight(model, a, b)),
                                 ("transit", transit_pos, lambda a, b: check_joint_move(model, a, b, table_z))):
            if pos is None:
                continue
            q = model.ik(pos, quat, top, restarts=0, stay=0.01, fixed={5: HACK_J6})
            if q is None or check(q, top):
                reasons.append(f"{name}: can't be reached on this arm configuration")
                ok = False
                break
            moves.insert(0, (name, q))
            top = q
        if not ok:
            continue
        if hover_pos is None and transit_pos is None:      # --here: reconfigure in place
            problems = check_in_place(model, q_from, top)
            moves = [("reconfigure", q_pre)]
        else:
            problems = check_joint_move(model, q_from, top, None)
        jump = float(np.max(np.abs(wrap_deg(top - q_from))))
        if problems or jump > MAX_STEP_JUMP_DEG:
            reasons.append("; ".join(problems) or f"a joint moves {jump:.0f} deg to get there")
            continue
        return moves, descent
    raise Refused(f"impedance: none of {len(pres)} arm configurations with J6 at -67.6 deg can lower "
                  "the bowl straight down here" + (f" (e.g. {reasons[0]})" if reasons else "")
                  + " -- run without --impedance (touch sensing)")


def plan_place(model, q_now, record=None, here=False, max_drop=MAX_DROP, bowl_depth=BOWL_DEPTH,
               max_adjust=MAX_ADJUST_DEG, impedance=False):
    """The whole place as (moves [(name, q)], descent (zs, qs), info).

    Raises Refused with the reason if any part does not check out.
    """
    T_now = model.fk(q_now)
    heading = T_now[:3, 2].copy()
    heading[2] = 0.0
    if np.linalg.norm(heading) < 0.3:
        raise Refused("the hand points too steeply up or down to tell which way it faces")
    adjust_quat = hand_orientation(heading)
    adjust_deg = float(np.degrees((Rotation.from_matrix(T_now[:3, :3]) * Rotation.from_quat(adjust_quat).inv()).magnitude()))
    if adjust_deg > max_adjust:
        raise Refused(f"the hand is {adjust_deg:.0f} deg from level + camera-on-the-right "
                      f"(max {max_adjust:.0f} to correct with a bowl in it) -- re-grip it closer, "
                      "or with an EMPTY gripper pass --max-adjust")

    table_z = None if here else record["table_z"]
    final_planner = plan_final_impedance if impedance else plan_final
    moves = plan_segment(model, q_now, T_now[:3, 3], adjust_quat, table_z, "adjust")
    if here:
        start = T_now[:3, 3]
        floor_z = start[2] - max_drop
        approach = heading / np.linalg.norm(heading)
        final, descent = final_planner(model, moves[-1][1], adjust_quat, start, floor_z, None)
        moves += final
    else:
        spot = np.asarray(record["spot"])
        approach = approach_direction(spot)
        quat = hand_orientation(approach)
        safe_z = table_z + SAFE_HEIGHT
        if T_now[2, 3] < safe_z - 0.01:
            lift = T_now[:3, 3].copy()
            lift[2] = safe_z
            moves += plan_segment(model, moves[-1][1], lift, adjust_quat, table_z, "lift")
        floor_z = table_z + max(bowl_depth - PRESS_BELOW, MIN_TOOL_HEIGHT)
        above = tool_above_spot(spot, table_z, SAFE_HEIGHT)
        floor_z = max(floor_z, above[2] - MAX_DROP)
        if impedance:
            final, descent = plan_final_impedance(
                model, moves[-1][1], quat,
                pre_pos=tool_above_spot(spot, table_z, bowl_depth + PRE_GAP), floor_z=floor_z,
                table_z=table_z, hover_pos=tool_above_spot(spot, table_z, bowl_depth + HOVER_GAP),
                transit_pos=above)
        else:
            # Carry over to above the spot at SAFE_HEIGHT, then ONE continuous move straight
            # down from there until the table is felt (no stops on the way down).
            final, descent = plan_final(model, moves[-1][1], quat, pre_pos=above, floor_z=floor_z,
                                        table_z=table_z, transit_pos=above)
        moves += final
    info = dict(adjust_deg=adjust_deg, approach=approach, table_z=table_z, floor_z=floor_z,
                impedance=impedance)
    return moves, descent, info


def retreat_poses(ee_pos, R_wb=np.eye(3)):
    """Straight back along the hand from the arm's OWN reported tool pose (Kortex ee_pos:
    xyz + quat xyzw, arm base frame). Built from what the arm reports, not the URDF, so the
    Cartesian move keeps the hand exactly as it is -- the URDF and the arm differ by ~1 cm /
    a few deg, and a URDF-derived orientation would make the arm turn the hand while backing
    off. "Back" is horizontal in the level frame; returned in the base frame."""
    pos, quat = np.asarray(ee_pos[:3], dtype=float), np.asarray(ee_pos[3:7], dtype=float)
    heading = R_wb @ Rotation.from_quat(quat).as_matrix()[:, 2]
    heading[2] = 0.0
    heading = R_wb.T @ (heading / np.linalg.norm(heading))
    return [("retreat", pos - RETREAT * heading, quat)]


# ---------------------------------------------------------------------------
# Arm + camera (connect_arm comes from arm_backend.py)
# ---------------------------------------------------------------------------

def wait_still(arm, timeout_s=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        state = arm.get_state()
        if np.max(np.abs(np.asarray(state["velocity"], dtype=float))) < 1e-3:
            return state
        time.sleep(0.2)
    return arm.get_state()


def grab_frames(depth_topic, info_topic, color_topic, n=1, period=0.2, timeout_s=10.0):
    """(intrinsics, [(depth_m, color_or_None), ...]): n depth frames from the wrist camera, each a
    new image, at least `period` s apart; colour is the latest one at that moment."""
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image
    from table_detector_node import depth_to_meters, image_to_numpy

    rclpy.init()
    node = rclpy.create_node("place_bowl_look")
    got = {}
    node.create_subscription(Image, depth_topic, lambda m: got.__setitem__("depth", m), 5)
    node.create_subscription(CameraInfo, info_topic, lambda m: got.setdefault("info", m), 5)
    if color_topic:
        node.create_subscription(Image, color_topic, lambda m: got.__setitem__("color", m), 5)
    frames, last, t_last = [], None, -np.inf
    t0 = time.time()
    try:
        while len(frames) < n and time.time() - t0 < timeout_s + n * period:
            rclpy.spin_once(node, timeout_sec=0.05)
            depth = got.get("depth")
            if depth is None or "info" not in got or depth is last or time.time() - t_last < period:
                continue
            if color_topic and "color" not in got and time.time() - t0 < 1.0:
                continue            # give colour a moment to arrive
            frames.append((depth, got.get("color")))
            last, t_last = depth, time.time()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not frames:
        sys.exit(f"No depth/camera_info within {timeout_s:.0f} s on {depth_topic}, {info_topic}. "
                 "Is the camera running? Check: ros2 topic list | grep camera")
    if len(frames) < n:
        print(f"only {len(frames)} of {n} frames arrived")
    K = got["info"].k
    intr = dict(fx=K[0], fy=K[4], cx=K[2], cy=K[5])
    return intr, [(depth_to_meters(d), image_to_numpy(c) if c is not None else None) for d, c in frames]


def self_distance(points, model, q):
    """Distance from each point to our own gripper (segment flange -> fingertip) at joints q."""
    a = model.fk(q, "end_effector_link")[:3, 3]
    b = model.fk(q, "finger_tip")[:3, 3]
    ab = b - a
    s = np.clip((points - a) @ ab / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(points - (a + s[:, None] * ab), axis=1)


# ---------------------------------------------------------------------------
# look
# ---------------------------------------------------------------------------

LOOK_RADIUS = BOWL_RADIUS + LIP_WIDTH               # room for the whole lip ...
LOOK_EDGE_MARGIN = max(EDGE_MARGIN - LIP_WIDTH, 0.0)  # ... the table edge stays EDGE_MARGIN from the body


def detect_frame(depth_m, intr, R, t, min_height=MIN_TABLE_HEIGHT, max_plane_dist=MAX_PLANE_DIST,
                 verbose=False):
    """table_detect on one frame, with the spot limited to the reach band. R, t: camera -> level frame."""
    rmin, rmax = REACH_BAND

    def reachable(points_cam):
        base = points_cam @ R.T + t
        r = np.hypot(base[:, 0], base[:, 1])
        return (r >= rmin) & (r <= rmax)

    # Room for the whole lip: objects stay 2 cm from the lip's edge; the table edge stays the
    # usual EDGE_MARGIN from the bowl body (the lip may overhang toward it).
    return detect_table(depth_m, intr, up=R.T @ UP, verbose=verbose, cam_to_base=(R, t),
                        min_height=min_height, max_plane_dist=max_plane_dist,
                        allowed=reachable, bowl_radius=LOOK_RADIUS, edge_margin=LOOK_EDGE_MARGIN)


def save_obstacles(depth_m, intr, result, R, t, model, q):
    """Everything standing on the table (level frame), for the hand-path check at place time."""
    points, pixels = backproject(depth_m, **intr, stride=1)
    obstacle = result["obstacle_mask"][pixels[:, 0], pixels[:, 1]]
    obstacles = points[obstacle] @ R.T + t
    # The wrist camera sees its own gripper; those points are not on the table.
    obstacles = obstacles[self_distance(obstacles, model, q) > SELF_RADIUS]
    if len(obstacles) > 50000:
        obstacles = obstacles[np.random.default_rng(0).choice(len(obstacles), 50000, replace=False)]
    np.save(OBSTACLE_FILE, obstacles)


def choose_spot(frames=10, period=0.2, window=True, depth_topic=None, info_topic=None, color_topic=None,
                min_height=MIN_TABLE_HEIGHT, max_plane_dist=MAX_PLANE_DIST):
    """Scan `frames` times at the scan pose, average the spots (outliers dropped), show the result
    and ask Confirm / Rescan / Cancel. Confirmed: saved to SPOT_FILE and returned as a dict, e.g.
      {"x": 0.851, "y": 0.094, "table_z": 0.132, "frame": "level", "spread_cm": 0.6, "n_used": 9,
       "confirmed": True, "spot": [x, y, z], "time": ..., ...}
    Cancelled: None, nothing saved. Nothing moves. The arm must be still, gripper empty."""
    depth_topic = depth_topic or f"{CAMERA_NS}/aligned_depth_to_color/image_raw"
    info_topic = info_topic or f"{CAMERA_NS}/aligned_depth_to_color/camera_info"
    color_topic = f"{CAMERA_NS}/color/image_raw" if color_topic is None else color_topic
    try:
        arm = connect_arm()
    except OSError as e:
        sys.exit(f"Cannot reach the arm ({e}) -- the joint angles are needed to place the camera.")
    model = ArmModel()
    calib = load_calibration()
    tilt_mount = np.degrees(np.arccos(np.clip(model.up_base @ UP, -1, 1)))
    print(f"level frame: arm base tilted {tilt_mount:.1f} deg"
          + ("" if MOUNT_FILE.exists() else " (no mount.json -- run `place_bowl.py level` if the arm is tilted)"))
    SPOT_DIR.mkdir(parents=True, exist_ok=True)
    rmin, rmax = REACH_BAND

    while True:
        q = np.asarray(wait_still(arm)["position"], dtype=float)
        print(f"\nscanning {frames} frames, {period:.1f} s apart ...")
        intr, shots = grab_frames(depth_topic, info_topic, color_topic, n=frames, period=period)
        q_after = np.asarray(arm.get_state()["position"], dtype=float)
        if np.max(np.abs(wrap_deg(q_after - q))) > 0.5:
            sys.exit("The arm moved while the frames were taken -- hold it still and retry.")
        T_base_cam = model.fk(q, "end_effector_link") @ calib
        R, t = T_base_cam[:3, :3], T_base_cam[:3, 3]
        print(f"camera at {np.round(t, 3).tolist()} m in the level frame, "
              f"looking {np.degrees(np.arccos(np.clip(-R[2, 2], -1, 1))):.0f} deg off straight down")

        tables, cands = [], []       # frames that found the table / that also found a spot
        for i, (depth_m, _color) in enumerate(shots):
            t0 = time.time()
            result = detect_frame(depth_m, intr, R, t, min_height, max_plane_dist)
            took = f"({time.time() - t0:.1f} s)"
            if result is None:
                print(f"  frame {i + 1:2d}: no table {took}")
                continue
            tilt = float(np.degrees(np.arccos(np.clip(abs((R @ result["normal"])[2]), -1, 1))))
            table_z = float(np.median(result["points"] @ R[2] + t[2]))
            if tilt > MAX_TABLE_TILT_DEG:
                print(f"  frame {i + 1:2d}: table plane {tilt:.1f} deg from level -- wrong plane? {took}")
                continue
            tables.append(dict(i=i, result=result, table_z=table_z, tilt=tilt))
            pl = result["placement"]
            if pl is None:
                print(f"  frame {i + 1:2d}: table z={table_z:.3f}, no room: {result['placement_reason']} {took}")
                continue
            base = R @ pl["point"] + t
            cands.append(dict(i=i, result=result, cam=pl["point"], base=base))
            print(f"  frame {i + 1:2d}: table z={table_z:.3f}, spot x={base[0]:.3f} y={base[1]:.3f} {took}")
        if not tables:
            depth_m, color = shots[len(shots) // 2]
            write_image(str(SPOT_DIR / "overlay.png"), _base_image(depth_m, color))
            sys.exit(f"No table found in any frame. Picture: {SPOT_DIR / 'overlay.png'}")

        vote = combine_spots([c["base"] for c in cands])
        ok, reason = vote["ok"], vote["reason"]
        spot_base = spot_cam = None
        clear = 0.0
        if ok:
            spot_base = vote["spot"]
            spot_cam = R.T @ (spot_base - t)
            # Shown on, and checked against, the frame whose own spot is nearest the average.
            ref = min((c for c, keep in zip(cands, vote["inliers"]) if keep),
                      key=lambda c: np.linalg.norm(c["base"][:2] - spot_base[:2]))
            clear = grid_clearance(spot_cam, ref["result"]["placement"])
            need = LOOK_RADIUS + min(MARGIN, LOOK_EDGE_MARGIN)
            r = float(np.hypot(*spot_base[:2]))
            if clear < need:
                ok, reason = False, (f"the averaged spot is only {clear * 100:.1f} cm clear (need "
                                     f"{need * 100:.1f}) -- the frames' spots surround something")
            elif not rmin <= r <= rmax:
                ok, reason = False, f"the averaged spot is {r:.2f} m from the base (reach {rmin:.2f}-{rmax:.2f})"
        else:
            ref = cands[len(cands) // 2] if cands else tables[len(tables) // 2]
        table_z = float(np.median([f["table_z"] for f in tables]))
        tilt = float(np.median([f["tilt"] for f in tables]))

        res = ref["result"]
        depth_m, color = shots[ref["i"]]
        base_img = overlay_image(_base_image(depth_m, color), res["mask"], obstacle_mask=res["obstacle_mask"])
        axes = res["placement"]["axes"] if res["placement"] is not None else plane_axes(res["normal"])
        n_in = int(vote["inliers"].sum())
        if ok:
            lines = [f"spot x={spot_base[0]:.3f} y={spot_base[1]:.3f} m, {np.hypot(*spot_base[:2]):.2f} m from base",
                     f"{n_in} of {len(cands)} spots agree ({frames} frames), spread {vote['spread'] * 100:.1f} cm",
                     f"table z={table_z:.3f} m, tilt {tilt:.1f} deg, {clear * 100:.1f} cm clear"]
        else:
            lines = textwrap.wrap(f"NOT USABLE: {reason}", 70) + [
                f"{len(cands)} of {frames} frames found a spot; R = rescan, Esc = cancel"]
        img = confirm_image(base_img, intr, axes, [c["cam"] for c in cands], vote["inliers"],
                            spot_cam, LOOK_RADIUS, lines, can_confirm=ok)
        write_image(str(SPOT_DIR / "overlay.png"), img)
        print("\n".join(lines))
        print(f"picture: {SPOT_DIR / 'overlay.png'} (grey = each frame's spot, red X = outlier, "
              "green = the spot)")

        choice = ask(img, can_confirm=ok, window=window)
        if choice == "rescan":
            continue
        if choice == "cancel":
            print("Cancelled -- nothing saved.")
            return None

        save_obstacles(depth_m, intr, res, R, t, model, q)
        record = dict(time=time.time(), time_str=time.strftime("%Y-%m-%d %H:%M:%S"),
                      x=float(spot_base[0]), y=float(spot_base[1]), table_z=table_z, frame="level",
                      spread_cm=round(vote["spread"] * 100, 2), n_used=n_in, n_found=len(cands),
                      n_frames=frames, confirmed=True, clearance=clear,
                      spot=[float(v) for v in spot_base], up_base=model.up_base.tolist(),
                      joints=q.tolist(), table_tilt_deg=tilt, bowl_radius=LOOK_RADIUS,
                      candidates=[[round(float(v), 4) for v in c["base"]] for c in cands],
                      inliers=[bool(v) for v in vote["inliers"]])
        SPOT_FILE.write_text(json.dumps(record, indent=2))
        print(f"confirmed, saved {SPOT_FILE}")
        return record


def _base_image(depth_m, color):
    return color if color is not None and color.shape[:2] == depth_m.shape else depth_to_color(depth_m)


def look(args):
    record = choose_spot(frames=args.frames, period=args.period, window=not args.no_window,
                         depth_topic=args.depth_topic, info_topic=args.info_topic,
                         color_topic=args.color_topic, min_height=args.min_height,
                         max_plane_dist=args.max_plane_dist)
    if record is None:
        sys.exit(1)
    print("Grip the bowl, then: python3 place_bowl.py place")


# ---------------------------------------------------------------------------
# place
# ---------------------------------------------------------------------------

def print_plan(model, moves, descent, info):
    table_z = info["table_z"]
    print(f"\n{'step':10s} {'tool x':>7s} {'y':>7s} {'z':>7s}  {'above table':>11s}  {'bowl tilt':>9s}")
    for name, q in moves:
        T = model.fk(q)
        above = "" if table_z is None else f"{T[2, 3] - table_z:9.3f} m"
        print(f"{name:10s} {T[0, 3]:7.3f} {T[1, 3]:7.3f} {T[2, 3]:7.3f}  {above:>11s}  "
              f"{bowl_tilt_deg(T[:3, :3]):6.1f} deg")
    zs, qs = descent
    floor = "" if table_z is None else f" ({(info['floor_z'] - table_z) * 100:.0f} cm above the table)"
    if info["impedance"]:
        print(f"lower      joint impedance from z={zs[0]:.3f} down to at most {zs[-1]:.3f}{floor}, "
              f"J6 held at {np.degrees(qs[0][5]):.1f} deg (the controller's model: -67.6)")
    else:
        print(f"lower      one continuous move at {SLIDE_SPEED * 100:.0f} cm/s from z={zs[0]:.3f} down to at most "
              f"{zs[-1]:.3f}{floor} ({(zs[0] - zs[-1]) * 100:.0f} cm), stopping when it feels the table; "
              "then opens and pulls out (no prompt)")
    print(f"adjust turns the hand {info['adjust_deg']:.1f} deg")


def joint_move(arm, q, name):
    """Position-controlled joint move + arrival check (like scan_pose.py)."""
    current = np.asarray(arm.get_state()["position"], dtype=float)
    raw = np.degrees(q - current)
    for i in CONTINUOUS:
        if abs(raw[i]) > 180.0:
            print(f"WARNING: J{i + 1} is {raw[i]:+.1f} deg away as raw numbers -- watch it "
                  "doesn't spin the long way.")
    print(f"-> {name}")
    arm.set_joint_position([float(v) for v in q])
    final = np.asarray(wait_still(arm)["position"], dtype=float)
    residual = np.max(np.abs(wrap_deg(q - final)))
    if residual > ARRIVE_TOL_DEG:
        raise Refused(f"STOPPED: {name} is off by {residual:.2f} deg")


def read_torques(arm, n=TORQUE_SAMPLES, spread=False):
    """Mean joint torques over n readings ~20 ms apart (and their spread, if asked)."""
    samples = []
    for _ in range(n):
        samples.append(np.asarray(arm.get_state()["effort"], dtype=float))
        time.sleep(0.02)
    samples = np.array(samples)
    return (samples.mean(axis=0), samples.std(axis=0)) if spread else samples.mean(axis=0)


def touch_down(arm, descent):
    """2 mm joint steps down until the joint torques say the bowl is on the table.
    Returns the step it stopped at; raises Refused if it reaches the bottom untouched."""
    zs, qs = descent
    # Let the torques settle after the moves that got here (reading too soon = a fake "jump").
    wait_still(arm, timeout_s=3.0)
    time.sleep(FIRST_SETTLE_S)
    start, noise = read_torques(arm, n=2 * TORQUE_SAMPLES, spread=True)
    noise = noise[TOUCH_JOINTS] / np.sqrt(TORQUE_SAMPLES)     # noise of a TORQUE_SAMPLES mean
    step_lim = np.maximum(STEP_TORQUE, NOISE_FACTOR * noise)
    total_lim = np.maximum(TOTAL_TORQUE, NOISE_FACTOR * noise)
    prev = start
    print(f"feeling for the table in {PATH_STEP * 1000:.0f} mm steps; torque noise J2/J4/J6 "
          f"{np.round(noise, 3).tolist()} Nm; contact = a change over {np.round(step_lim, 2).tolist()} Nm "
          f"in one step or {np.round(total_lim, 2).tolist()} Nm in total, still there {CONFIRM_S} s later")

    history = [start]           # torques at the last few steps (rolling reference)

    def over(now):
        earlier = history[-min(len(history), 5)]       # ~1 cm earlier, not the start (gravity drift)
        step, total = np.abs(now - prev)[TOUCH_JOINTS], np.abs(now - earlier)[TOUCH_JOINTS]
        return bool(np.any(step > step_lim) or np.any(total > total_lim))

    for k in range(1, len(qs)):
        arm.set_joint_position([float(v) for v in qs[k]])
        wait_still(arm, timeout_s=3.0)
        time.sleep(SETTLE_S)
        now = read_torques(arm)
        step, total = (now - prev)[TOUCH_JOINTS], (now - start)[TOUCH_JOINTS]
        print(f"  z={zs[k]:.3f}  change this step J2/J4/J6 {np.round(step, 2).tolist()} Nm, "
              f"since start {np.round(total, 2).tolist()} Nm")
        if over(now):
            time.sleep(CONFIRM_S)          # same pose: a real contact stays, a spike goes away
            again = read_torques(arm)
            if over(again):
                print(f"contact at tool z={zs[k]:.3f} (confirmed: since start "
                      f"{np.round((again - start)[TOUCH_JOINTS], 2).tolist()} Nm)")
                return k
            print("  (spike, gone on re-reading -- carrying on)")
            now = again
        prev = now
        history.append(now)
    raise Refused("STOPPED: reached the lowest planned height without feeling the table "
                  "(bowl shallower than --bowl-depth, or the table lower than measured?)")


def slide_down(arm, descent):
    """One slow continuous joint trajectory down the planned line while the torques are
    watched on a second connection; stop_action the moment they jump, then confirm standing
    still. Returns the tool z at contact. Falls back to touch_down (steps) if the arm rejects
    the trajectory."""
    import threading
    zs, qs = descent
    every = max(1, int(round(SLIDE_SPEED * 0.5 / PATH_STEP)))
    watcher = connect_arm()          # second connection: reads + stop while the move blocks
    wait_still(arm, timeout_s=3.0)
    time.sleep(FIRST_SETTLE_S)
    static0, noise = read_torques(watcher, n=2 * TORQUE_SAMPLES, spread=True)
    noise = noise[TOUCH_JOINTS]
    move_lim = np.maximum(MOVE_TORQUE, NOISE_FACTOR * noise / np.sqrt(WINDOW))
    total_lim = np.maximum(TOTAL_TORQUE, NOISE_FACTOR * noise / np.sqrt(TORQUE_SAMPLES))
    print(f"lowering continuously at {SLIDE_SPEED * 100:.0f} cm/s; torque noise J2/J4/J6 "
          f"{np.round(noise, 3).tolist()} Nm; stop on a change over {np.round(move_lim, 2).tolist()} Nm "
          f"for {HITS} readings")
    k0 = 0
    for attempt in range(MAX_RESUMES + 1):
        idx = list(range(k0 + every, len(qs), every))
        if not idx or idx[-1] != len(qs) - 1:
            idx.append(len(qs) - 1)
        traj = [[float(v) for v in qs[i]] for i in idx]
        result = {}
        mover = threading.Thread(target=lambda: result.update(ok=arm.set_joint_trajectory(traj)),
                                 daemon=True)
        mover.start()
        t0, readings, hits, stopped, ref = time.time(), [], 0, False, None
        while mover.is_alive():
            t = time.time()
            readings.append((t, np.asarray(watcher.get_state()["effort"], dtype=float)))
            window = [e for (tt, e) in readings if t - BASELINE_LAG_S - BASELINE_S <= tt <= t - BASELINE_LAG_S]
            if t - t0 > 0.25 + BASELINE_LAG_S + BASELINE_S and len(window) >= 5:
                ref = np.mean(window, axis=0)               # torques from 0.3-0.7 s ago
                now = np.mean([e for (_, e) in readings[-WINDOW:]], axis=0)
                hits = hits + 1 if np.any(np.abs(now - ref)[TOUCH_JOINTS] > move_lim) else 0
                if hits >= HITS:
                    watcher.stop_action()
                    stopped = True
                    break
            time.sleep(POLL_S)
        mover.join(timeout=10.0)
        if not stopped and not result.get("ok", False) and attempt == 0 and k0 == 0:
            print("the arm rejected the slow trajectory -- falling back to 2 mm steps")
            return zs[touch_down(arm, descent)]
        state = wait_still(watcher, timeout_s=3.0)
        q_now = np.asarray(state["position"], dtype=float)
        k_now = int(np.argmin([np.max(np.abs(wrap_deg(q - q_now))) for q in qs]))
        if not stopped:
            raise Refused("STOPPED: reached the lowest planned height without feeling the table "
                          "(bowl shallower than --bowl-depth, or the table lower than measured?)")
        time.sleep(SETTLE_S)
        static = read_torques(watcher)
        # Compare with the torques from just BEFORE the trigger, not the start of the move.
        change = (static - ref)[TOUCH_JOINTS]
        print(f"  stopped at z={zs[k_now]:.3f}: change vs just before the stop, standing still, "
              f"J2/J4/J6 {np.round(change, 2).tolist()} Nm")
        if np.any(np.abs(change) > total_lim):
            print(f"contact at tool z={zs[k_now]:.3f}")
            return zs[k_now]
        print("  (false alarm: gone standing still -- carrying on down)")
        k0 = k_now
    raise Refused("STOPPED: too many false alarms while lowering -- try --steps")


def lower_compliant(arm, model, descent):
    """Joint-space impedance lowering (the lab's joint compliant mode) with J6 at HACK_J6.
    Leaves compliant mode on the way out, whatever happens."""
    zs, qs = descent
    actual = float(np.asarray(arm.get_state()["position"], dtype=float)[5])
    for name, j6 in (("planned", float(qs[0][5])), ("actual", actual)):
        if abs(wrap_deg(j6 - HACK_J6)) > HACK_J6_TOL_DEG:
            raise Refused(f"STOPPED before compliant mode: {name} J6 is {np.degrees(j6):.1f} deg, but the "
                          f"lab's compliant controller models J6 at {np.degrees(HACK_J6):.1f} deg -- its "
                          "gravity compensation would drop the wrist")
    period = 1.0 / STREAM_HZ

    def hand_z():
        q = np.asarray(arm.get_state()["position"], dtype=float)
        q[5] = HACK_J6               # compliant get_state reports J6 as the fixed value anyway
        return model.fk(q)[2, 3]

    def target_at(z):
        """Joint target for tool height z, interpolated along the planned descent; 6 values
        (J6 is not commanded in compliant mode)."""
        k = np.clip(np.searchsorted(-zs, -z), 1, len(zs) - 1)
        s = np.clip((zs[k - 1] - z) / (zs[k - 1] - zs[k]), 0.0, 1.0)
        q = qs[k - 1] + s * wrap(qs[k] - qs[k - 1])
        return [float(v) for i, v in enumerate(q) if i != 5]

    z_start = hand_z()
    print("switching to joint compliant mode ...")
    arm.switch_to_joint_compliant_mode()
    try:
        for _ in range(int(1.0 * STREAM_HZ)):        # hold still while it takes over
            arm.compliant_set_joint_position(target_at(z_start))
            time.sleep(period)
            if z_start - hand_z() > MAX_SAG:
                raise Refused(f"STOPPED: the hand dropped {(z_start - hand_z()) * 100:.1f} cm "
                              "when compliant mode took over")
        z_now = hand_z()
        print(f"lowering at {LOWER_SPEED * 100:.0f} cm/s ...")
        target, history = z_now, []
        deadline = time.time() + 3 * (z_now - zs[-1]) / LOWER_SPEED + 5.0
        while True:
            z_now = hand_z()
            history.append((time.time(), z_now))
            old = [z for t_, z in history if t_ <= time.time() - STALL_WINDOW_S]
            if old and old[-1] - z_now < STALL_DIST:          # the hand has stopped going down
                if z_now - target > LEAD / 4:                  # ... while its target is below it
                    print(f"contact at tool z={z_now:.3f}")
                    break
                if target <= zs[-1] + 1e-4:
                    raise Refused("STOPPED: reached the lowest planned height without touching the table")
            if time.time() > deadline:
                raise Refused("STOPPED: lowering took too long")
            target = max(target - LOWER_SPEED * period, z_now - LEAD, zs[-1])
            arm.compliant_set_joint_position(target_at(target))
            time.sleep(period)
        press = max(z_now - PRESS, zs[-1])
        for _ in range(int(HOLD_S * STREAM_HZ)):
            arm.compliant_set_joint_position(target_at(press))
            time.sleep(period)
    finally:
        print("switching back to position control ...")
        arm.switch_out_of_compliant_mode()


def place_at(spot, execute=False, **options):
    """Place the held bowl at `spot` -- the dict choose_spot() returns -- as `place` would:
        spot = choose_spot()
        if spot:
            place_at(spot, execute=True)
    options: any `place` flag by its attribute name (yes=True, max_drop=0.3, ...).
    Like the command line, it exits (SystemExit) with the reason when it refuses."""
    global ARGS
    ARGS = build_parser().parse_args(["place"])
    ARGS.execute = execute
    for name, value in options.items():
        if not hasattr(ARGS, name):
            raise TypeError(f"place_at: unknown option {name!r}")
        setattr(ARGS, name, value)
    if ARGS.here:
        raise TypeError("place_at: --here takes no spot; use `place --here`")
    place(ARGS, record=spot)


def manual_spot(x, y, table_z, up_base, source):
    """A spot record for a target given by hand (`mark`, `place --at`) instead of found by `look`.
    x, y: bowl centre, table_z: table height, all in the level frame (m). Confirmed: a person chose it."""
    return dict(time=time.time(), time_str=time.strftime("%Y-%m-%d %H:%M:%S"), x=float(x), y=float(y),
                table_z=float(table_z), frame="level", confirmed=True, source=source,
                spot=[float(x), float(y), float(table_z)], up_base=np.asarray(up_base, dtype=float).tolist())


def mark(args):
    """Save where the fingertip is now as the spot: touch the target on the table with it first."""
    try:
        arm = connect_arm()
    except OSError as e:
        sys.exit(f"Cannot reach arm_server ({e}).")
    model = ArmModel()
    q = np.asarray(wait_still(arm)["position"], dtype=float)
    tip = model.fk(q, "finger_tip")[:3, 3]
    record = manual_spot(tip[0], tip[1], tip[2], model.up_base, "mark")
    print(f"fingertip at x={tip[0]:.3f} y={tip[1]:.3f} z={tip[2]:.3f} m (level frame), "
          f"{np.hypot(*tip[:2]):.2f} m from the base axis")
    SPOT_DIR.mkdir(parents=True, exist_ok=True)
    SPOT_FILE.write_text(json.dumps(record, indent=2))
    print(f"saved {SPOT_FILE}: the bowl centre goes here, table height {tip[2]:.3f} m.\n"
          f"Same spot later: place --at {tip[0]:.3f} {tip[1]:.3f} --table-z {tip[2]:.3f}")


def check_spot(record, max_age, require_confirmed=True):
    """Exit with the reason if a saved spot must not be used."""
    age_min = (time.time() - record["time"]) / 60
    print(f"spot from {record['time_str']} ({age_min:.0f} min ago): "
          f"{np.round(record['spot'], 3).tolist()} m, table z {record['table_z']:.3f} m"
          + (f", {record['n_used']} frames, spread {record['spread_cm']:.1f} cm" if "n_used" in record else ""))
    if require_confirmed and not record.get("confirmed"):
        sys.exit("The spot wasn't confirmed -- run `place_bowl.py look` and click Confirm "
                 "(or pass --no-confirm).")
    up_then = np.asarray(record.get("up_base", UP), dtype=float)
    if np.degrees(np.arccos(np.clip(up_then @ load_mount()[1], -1, 1))) > 0.5:
        sys.exit("The spot was found before the last `level` -- run look again.")
    if age_min > max_age:
        sys.exit(f"Spot is older than {max_age:.0f} min -- the base or table may have moved. "
                 "Run look again (or pass --max-age).")


def place(args, record=None):
    """record: the spot to use (place_at); None reads SPOT_FILE (or none with --here)."""
    model = ArmModel()
    if args.impedance:
        tilt = np.degrees(np.arccos(np.clip(model.up_base @ UP, -1, 1)))
        if not MOUNT_FILE.exists():
            sys.exit("--impedance: run `place_bowl.py level` first -- the lab's compliant controller "
                     "assumes an upright arm base, so the mount tilt has to be known.")
        if tilt > 2.0:
            sys.exit(f"--impedance refused: the arm base is tilted {tilt:.0f} deg, and the lab's compliant "
                     "controller computes gravity as if it were upright (kinova.py gravity(): pinocchio "
                     "default, -z of the base). Its gravity compensation would push the arm sideways with "
                     f"~{np.sin(np.radians(tilt)) * 100:.0f}% of its weight. Use touch sensing (no --impedance).")
    if args.here:
        record = None
    elif args.at is not None:
        table_z = args.table_z
        if table_z is None:
            if not SPOT_FILE.exists():
                sys.exit("--at needs the table height: pass --table-z, or run `mark` or `look` once first.")
            table_z = json.loads(SPOT_FILE.read_text())["table_z"]
            print(f"table height {table_z:.3f} m, from the last saved spot")
        record = manual_spot(args.at[0], args.at[1], table_z, model.up_base, "at")
    else:
        if record is None:
            if not SPOT_FILE.exists():
                sys.exit(f"No saved spot ({SPOT_FILE}). Run: python3 place_bowl.py look  (or use --here)")
            record = json.loads(SPOT_FILE.read_text())
        check_spot(record, args.max_age, require_confirmed=not args.no_confirm)

    try:
        arm = connect_arm()
    except OSError as e:
        sys.exit(f"Cannot reach arm_server ({e}). The plan starts from the arm's current pose.")
    state = wait_still(arm)
    q_now = np.asarray(state["position"], dtype=float)
    reported = np.asarray(state["ee_pos"][:3], dtype=float)
    modelled = model.fk(q_now, base=True)[:3, 3]
    off = float(np.linalg.norm(reported - modelled))
    print(f"tool frame: arm says {np.round(reported, 3).tolist()}, URDF says "
          f"{np.round(modelled, 3).tolist()} ({off * 100:.1f} cm apart)")
    if off > TOOL_FRAME_TOL:
        sys.exit("The arm's tool frame doesn't match the URDF tool_frame. Check the tool "
                 "configuration in the Kinova web app.")
    gripper = float(state.get("gripper_pos"))
    print(f"gripper: {gripper:.3f} (0 = open, 1 = closed)")
    if gripper < 0.05:
        sys.exit("The gripper is fully open -- it isn't holding the bowl.")

    try:
        moves, descent, info = plan_place(model, q_now, record, here=args.here, max_drop=args.max_drop,
                                          bowl_depth=args.bowl_depth, max_adjust=args.max_adjust,
                                          impedance=args.impedance)
        if record is not None and record.get("source") in ("mark", "at"):
            r = float(np.hypot(*record["spot"][:2]))
            print(f"target given by hand, {r:.2f} m from the base axis -- no obstacle check, keep it clear"
                  + ("" if REACH_BAND[0] <= r <= REACH_BAND[1] else
                     f" (outside the swept reach {REACH_BAND[0]:.2f}-{REACH_BAND[1]:.2f}; the plan decides)"))
        elif record is not None:
            obstacles = np.load(OBSTACLE_FILE) if OBSTACLE_FILE.exists() else np.zeros((0, 3))
            blocked = corridor_obstacles(obstacles, np.asarray(record["spot"]), record["table_z"],
                                         info["approach"])
            if len(blocked):
                raise Refused(f"{len(blocked)} obstacle points where the hand comes down")
    except Refused as e:
        sys.exit(f"REFUSED: {e}")
    print_plan(model, moves, descent, info)

    if not args.execute:
        print("\nDRY RUN -- all checks passed, nothing commanded. Add --execute to move.")
        return

    try:
        speed = arm.get_speed()
    except Exception as e:
        sys.exit(f"Arm is not accepting commands: {e}\nStart bulldog_bypass.py (single-machine rig).")
    if speed != REQUIRED_SPEED:
        sys.exit(f"Speed preset is '{speed}', not '{REQUIRED_SPEED}'. Run:\n"
                 f"  python3 ~/feeding-deployment/scripts/session/arm_set_speed.py {REQUIRED_SPEED}")
    confirm("Area clear, bowl gripped, hand on the e-stop? [ENTER] to start")

    try:
        for name, q in moves:
            joint_move(arm, q, name)
        T = model.fk(np.asarray(arm.get_state()["position"], dtype=float))
        if bowl_tilt_deg(T[:3, :3]) > 1.0:
            raise Refused(f"STOPPED: bowl is {bowl_tilt_deg(T[:3, :3]):.1f} deg from level after the moves")
        if args.impedance:
            lower_compliant(arm, model, descent)
        elif args.steps:
            touch_down(arm, descent)
        else:
            slide_down(arm, descent)
    except Refused as e:
        sys.exit(str(e))

    # Contact was confirmed standing still (a false alarm carries on down instead), so let go
    # and pull out right away -- no prompt.
    print("opening the gripper ...")
    arm.open_gripper()
    time.sleep(0.5)
    for name, pos, quat in retreat_poses(arm.get_state()["ee_pos"], model.R_wb):
        print(f"-> {name}")
        arm.set_ee_pose(pos.tolist(), quat.tolist())
        wait_still(arm)
    print("Done: bowl placed, hand clear.")


def level(args):
    """Record true up from the hand, held truly level with the camera on the right."""
    if args.clear:
        MOUNT_FILE.unlink(missing_ok=True)
        print(f"removed {MOUNT_FILE}: the arm base's z counts as up again")
        return
    try:
        arm = connect_arm()
    except OSError as e:
        sys.exit(f"Cannot reach arm_server ({e}).")
    R_tool = Rotation.from_quat(np.asarray(wait_still(arm)["ee_pos"][3:7], dtype=float)).as_matrix()
    up_base = R_tool[:, 0]        # level hand, camera on the right -> tool x points straight up
    tilt = np.degrees(np.arccos(np.clip(up_base @ UP, -1.0, 1.0)))
    if up_base[2] < 0:
        sys.exit("The hand's tool x points down: the camera is on the LEFT. Turn the hand so the "
                 "camera is on the right (seen from behind), level, and retry.")
    if tilt > 30.0:
        sys.exit(f"That makes the arm {tilt:.0f} deg tilted -- too much to be the mount. Is the hand "
                 "really level, with the camera on the right?")
    toward = np.array([up_base[0], up_base[1], 0.0])
    toward_deg = np.degrees(np.arctan2(toward[1], toward[0]))
    print(f"true up in the arm base frame: {np.round(up_base, 4).tolist()}")
    print(f"=> the arm base is tilted {tilt:.1f} deg (true up leans toward base "
          f"{toward_deg:.0f} deg, where x = 0 and y = 90)")
    SPOT_DIR.mkdir(parents=True, exist_ok=True)
    MOUNT_FILE.write_text(json.dumps(dict(up_base=up_base.tolist(), tilt_deg=float(tilt),
                                          time_str=time.strftime("%Y-%m-%d %H:%M:%S")), indent=2))
    print(f"saved {MOUNT_FILE}. `look` and `place` now use this as level / straight down. "
          "Run look again for a new spot.")


def confirm(prompt):
    if ARGS.yes:
        return
    try:
        input(f"\n{prompt} (Ctrl-C to abort) ")
    except (KeyboardInterrupt, EOFError):
        sys.exit("\nAborted.")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_look = sub.add_parser("look", help="scan, average, confirm by clicking, save the spot (no motion)")
    p_look.add_argument("--frames", type=int, default=LOOK_FRAMES,
                        help="frames to scan and average (default %(default)s)")
    p_look.add_argument("--period", type=float, default=LOOK_PERIOD,
                        help="s between frames (default %(default).1f)")
    p_look.add_argument("--no-window", action="store_true",
                        help="confirm in the terminal instead of a window (the picture is still saved)")
    p_look.add_argument("--depth-topic", default=f"{CAMERA_NS}/aligned_depth_to_color/image_raw")
    p_look.add_argument("--info-topic", default=f"{CAMERA_NS}/aligned_depth_to_color/camera_info")
    p_look.add_argument("--color-topic", default=f"{CAMERA_NS}/color/image_raw")
    p_look.add_argument("--min-height", type=float, default=MIN_TABLE_HEIGHT,
                        help="lowest table height in arm_base_link, m (default %(default)s)")
    p_look.add_argument("--max-plane-dist", type=float, default=MAX_PLANE_DIST,
                        help="skip planes farther than this from the camera, m (default %(default).2f)")
    p_level = sub.add_parser("level", help="record true up: hold the hand truly level, camera on the right")
    p_level.add_argument("--clear", action="store_true", help="forget it (base z = up again)")
    sub.add_parser("mark", help="save the fingertip's position as the spot: touch the target on the "
                                "table with the fingertip first (no motion)")
    p_place = sub.add_parser("place", help="put the held bowl down (dry run unless --execute)")
    p_place.add_argument("--execute", action="store_true", help="actually move")
    p_place.add_argument("--yes", action="store_true", help="skip the [ENTER] prompts")
    p_place.add_argument("--here", action="store_true",
                         help="no spot: adjust, then lower straight down from where the hand is")
    p_place.add_argument("--at", type=float, nargs=2, metavar=("X", "Y"),
                         help="place the bowl centre here instead of the saved spot: level frame, m "
                              "(origin on the arm base axis, x forward, y left; `mark` prints them)")
    p_place.add_argument("--table-z", type=float,
                         help="--at: table height, m (default: from the last saved spot)")
    p_place.add_argument("--max-drop", type=float, default=MAX_DROP,
                         help="--here: lowest the lowering may go below the start, m (default %(default).2f)")
    p_place.add_argument("--steps", action="store_true",
                         help="lower in 2 mm steps (stop, read torques, repeat) instead of one slow move")
    p_place.add_argument("--impedance", action="store_true",
                         help="lower the last few cm with the lab's joint compliant mode instead of "
                              "touch sensing (puts J6 at -67.6 deg first; works in fewer places)")
    p_place.add_argument("--max-adjust", type=float, default=MAX_ADJUST_DEG,
                         help="largest squaring turn allowed, deg (default %(default).0f; raise it "
                              "only with an EMPTY gripper -- the turn tips a held bowl)")
    p_place.add_argument("--bowl-depth", type=float, default=BOWL_DEPTH,
                         help="lip to bowl bottom, m (default %(default).2f)")
    p_place.add_argument("--max-age", type=float, default=MAX_AGE_MIN,
                         help="refuse a saved spot older than this, min (default %(default).0f)")
    p_place.add_argument("--no-confirm", action="store_true",
                         help="accept a saved spot nobody clicked Confirm on")
    return parser


def main():
    global ARGS
    ARGS = build_parser().parse_args()
    ARGS.yes = getattr(ARGS, "yes", False)
    {"look": look, "level": level, "mark": mark, "place": place}[ARGS.cmd](ARGS)


ARGS = None

if __name__ == "__main__":
    main()
