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

The arm bring-up scripts (`arm_server.py`, `bulldog_bypass.py`, `scripts/session/arm_set_speed.py`) are
used from `~/feeding-deployment`; see "How to run it again" in `PROGRESS_2026-09-29.md`, but run from this
folder instead of `~/walter_table`.

`table_gaze_pos` in `vention.yaml` looks sideways with the current camera calibration; use the scan pose.
Placing details (why J6 matters in compliant mode, the reach ring, --bowl-depth) are in `place_bowl.py`'s docstring.
