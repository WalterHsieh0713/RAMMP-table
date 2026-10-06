# Table detection + bowl placement

Finds the table in one wrist-camera depth frame and picks a clear spot on it for the bowl.
Copied from `~/walter_table` on 2026-10-04 (history and results: `PROGRESS_2026-09-29.md`).
Nothing outside this folder is changed.

| File | What |
|---|---|
| `table_detect.py` | Pure numpy/scipy: table plane (RANSAC, floor rejected by distance / height) + free-space bowl spot. |
| `table_detector_node.py` | ROS 2 node: runs it live at 2 Hz, publishes `/table_detector/overlay` and RViz markers. |
| `scan_pose.py` | Moves the arm to the fixed table-scan pose (safety-checked; `--dry-run` sends nothing). |
| `scan_and_detect.sh` | Scan pose, then detector + overlay viewer, one command. |
| `save_frame.py` | Saves one depth/color frame and prints the intrinsics. |
| `test_placement.py` | Offline tests: `python3 test_placement.py`. |
| `place_bowl.py` | `look` (find + save a reachable spot) and `place` (bowl already gripped: square the hand, carry, lower with joint-space impedance, release). Dry run unless `--execute`. |
| `test_place_bowl.py` | Offline tests for the planner; `--sweep` maps where the arm can place. |
| `spot_vote.py` | `look`'s several-scan vote: drop outlying spots, average the rest, check the average is clear, and the Confirm / Rescan / Cancel window (terminal prompt without a display). |
| `test_spot_vote.py` | Offline tests for it, no pybullet needed; `--show` opens the window on a synthetic scan. |
| `arm_backend.py` | The one place that knows which arm stack is used (`TABLE_ARM_BACKEND`, default `feeding`) and what an arm object must provide. |

## Running on another machine (e.g. the Jetson under Sheppy)
Nothing to edit; export what differs:

| Variable | Default (rchi-cpu-5) |
|---|---|
| `TABLE_ARM_BACKEND` | `feeding`: `feeding-deployment`'s `arm_server.py` at `ARM_RPC_HOST` (127.0.0.1) |
| `TABLE_CAMERA_NS` | `/camera/wrist` (topic prefix for `place_bowl.py look`) |
| `TABLE_URDF` | `feeding-deployment/src/feeding_deployment/assets/robot/robot.urdf` |
| `TABLE_CALIB` | `~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib` |
| `TABLE_STATE_DIR` | `~/.table_place` (spot, mount tilt, overlay) |

If the Jetson commands the arm some other way than `arm_server.py`, add a backend to `arm_backend.py`.
The docstring lists the calls it must provide.

The arm bring-up scripts (`arm_server.py`, `bulldog_bypass.py`, `scripts/session/arm_set_speed.py`) are
used from `~/feeding-deployment`; see "How to run it again" in `PROGRESS_2026-09-29.md`, but run from this
folder instead of `~/walter_table`.

`table_gaze_pos` in `vention.yaml` looks sideways with the current camera calibration; use the scan pose.
Placing details (why J6 matters in compliant mode, the reach ring, --bowl-depth) are in `place_bowl.py`'s docstring.
