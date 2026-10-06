"""Move the Kinova Gen3 to the fixed table-scan pose, then exit.

Run this BEFORE table_detector_node.py -- the detector itself never moves the arm.

    python3 scan_pose.py --dry-run     # read the arm, print the command, send nothing
    python3 scan_pose.py               # asks for [ENTER] before moving
    python3 scan_pose.py --yes         # no prompt

Follows ~/feeding-deployment's preset scripts (retract.py, and goto_preset.py for the
ROS 2 rig): same RPC client, one set_joint_position() call, angles in RADIANS
(kinova.py move_angular converts to degrees itself). Plain Python -- no rospy/rclpy.

Needs, already running and pointed at the same host:
  - arm_server.py, plus bulldog_bypass.py on the single-machine rig (or bulldog on
    the lab NUC) -- without it the server refuses motion ("Bulldog is not running").
  - the "low" speed preset (30 deg/s): feeding-deployment/scripts/session/arm_set_speed.py low
Server address comes from ARM_RPC_HOST, defaulted to 127.0.0.1 for this
single-machine rig (rchi-cpu-5). For the lab NUC: ARM_RPC_HOST=192.168.1.3

The only software stop is Ctrl-C / killing arm_server -- keep a hand on the e-stop.
"""
import argparse
import sys
import time

import numpy as np

from arm_backend import connect_arm, describe   # which arm stack: TABLE_ARM_BACKEND

# Scan pose, J1..J7, RADIANS, each in [-pi, pi]. Read off the Kinova web UI as
# (353.787, 310.642, 181.649, 234.867, 2.240, 302.922, 91.168) deg in 0-360 form;
# signed: (-6.213, -49.358, -178.351, -125.133, 2.240, -57.078, 91.168) deg.
SCAN_POSE = [-0.10844, -0.86146, -3.11281, -2.18398, 0.03910, -0.99620, 1.59118]

CONTINUOUS = [0, 2, 4, 6]   # J1, J3, J5, J7 have no limits; J2/J4/J6 are limited
REQUIRED_SPEED = "low"
LOW_SPEED_DEG_S = 30.0      # kinova.py choose_from_speed_presets("low")
TOLERANCE_DEG = 1.0         # arrival check (move_angular's own check is 5 deg)


def wrap_deg(rad):
    """Angle difference in radians -> degrees in [-180, 180)."""
    return np.degrees((np.asarray(rad) + np.pi) % (2 * np.pi) - np.pi)


def fmt(deg):
    return "[" + ", ".join(f"{v:8.2f}" for v in deg) + "]"


parser = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--yes", action="store_true", help="skip the [ENTER] confirmation")
parser.add_argument("--dry-run", action="store_true", help="print the command, send nothing")
parser.add_argument("--max-jump", type=float, default=150.0,
                    help="refuse if any joint would travel more than this, deg (default 150)")
args = parser.parse_args()

target = np.asarray(SCAN_POSE, dtype=float)
print(f"arm           : {describe()}")
print(f"target  (deg) : {fmt(np.degrees(target))}")
print(f"target  (rad) : {SCAN_POSE}")

try:
    arm = connect_arm()
except OSError as e:
    msg = f"Cannot reach the arm: {describe()} ({e})."
    if args.dry_run:
        print(f"\n{msg}\nDRY RUN -- would send: set_joint_position({SCAN_POSE})")
        sys.exit(0)
    sys.exit(f"{msg}\nIs the arm stack running? (README: how to run it)")

state = arm.get_state()
current = np.asarray(state["position"], dtype=float)
gripper = float(state.get("gripper_pos"))
delta = wrap_deg(target - current)
raw_delta = np.degrees(target - current)

# Print the pose being LEFT too: arm_commands_log.txt is wiped on every arm_server
# restart, so this line is the only record of how to get back.
print(f"current (deg) : {fmt(np.degrees(current))}")
print(f"current (rad) : {[round(float(v), 5) for v in current]}")
print(f"delta   (deg) : {fmt(delta)}")
print(f"max joint move: {np.max(np.abs(delta)):.1f} deg  "
      f"(~{np.max(np.abs(delta)) / LOW_SPEED_DEG_S + 1.0:.0f} s at the '{REQUIRED_SPEED}' preset)")
print(f"gripper       : {gripper:.4f}  ({'open' if gripper < 0.2 else 'CLOSED'})")

# The command is not normalised anywhere on its way to the arm. For a continuous
# joint, if the raw difference is the long way round, say so, so it can be watched.
long_way = [i for i in CONTINUOUS if abs(raw_delta[i]) > 180.0]
for i in long_way:
    print(f"WARNING: J{i + 1} is {raw_delta[i]:+.1f} deg away as raw numbers but "
          f"{delta[i]:+.1f} deg the short way -- watch that it doesn't spin the long way.")

if gripper > 0.2:
    sys.exit("Gripper is CLOSED -- it may be holding something. Refusing to move.")
if np.max(np.abs(delta)) > args.max_jump:
    sys.exit(f"Move too large ({np.max(np.abs(delta)):.0f} deg > {args.max_jump}). "
             "Jog closer first, or pass --max-jump if this is intended.")

if args.dry_run:
    print(f"\nDRY RUN -- would send: set_joint_position({SCAN_POSE})\nNothing commanded.")
    sys.exit(0)

# Verify rather than set the speed, like the repo's own scripts: the preset is shared
# arm state that arm_set_speed.py owns. get_speed() also fails fast if bulldog is down.
try:
    speed = arm.get_speed()
except Exception as e:
    sys.exit(f"Arm is not accepting commands: {e}\n"
             "Start bulldog_bypass.py (single-machine rig) or bulldog (lab), then retry.")
if speed != REQUIRED_SPEED:
    sys.exit(f"Speed preset is '{speed}', not '{REQUIRED_SPEED}'. Run:\n"
             f"  python3 ~/feeding-deployment/scripts/session/arm_set_speed.py {REQUIRED_SPEED}")
print(f"speed         : {speed}")

if not args.yes:
    try:
        input("\nArea clear, hand on the e-stop? Press [ENTER] to move (Ctrl-C to abort) ")
    except (KeyboardInterrupt, EOFError):
        sys.exit("\nAborted -- nothing commanded.")

print("Moving to scan pose ...")
ok = arm.set_joint_position(SCAN_POSE)   # blocks until the arm reports done
for _ in range(50):   # let it settle before reading back
    if np.max(np.abs(np.asarray(arm.get_state()["velocity"], dtype=float))) < 1e-3:
        break
    time.sleep(0.2)

final = np.asarray(arm.get_state()["position"], dtype=float)
residual = wrap_deg(target - final)
print(f"final   (deg) : {fmt(np.degrees(final))}")
print(f"residual(deg) : {fmt(residual)}   max {np.max(np.abs(residual)):.2f}")

if np.max(np.abs(residual)) > TOLERANCE_DEG:
    bad = ", ".join(f"J{i + 1} {residual[i]:+.2f}" for i in np.flatnonzero(np.abs(residual) > TOLERANCE_DEG))
    sys.exit(f"DID NOT ARRIVE: off by more than {TOLERANCE_DEG} deg on {bad} "
             f"(server reported success={ok}).")
print(f"At scan pose (within {TOLERANCE_DEG} deg). Start the detector now.")
