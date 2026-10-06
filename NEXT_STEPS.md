# What to do next (updated 2026-10-05)

History: `PROGRESS_2026-09-29.md`, `PROGRESS_2026-10-05.md`. The design of the several-scan `look`: `PLAN_2026-10-05.md`.

## Done on 2026-10-05, offline only (not run on the arm yet, not committed)
- **`look` scans several times, averages, and asks before saving** (`spot_vote.py`, `place_bowl.py`):
  - `--frames 10` frames, `--period 0.2` s apart; each frame finds a spot;
  - outliers are dropped (more than 1.5 cm from the median, `OUTLIER_TOL`) and the rest averaged;
  - it refuses if fewer than 3 frames found a spot, fewer than 60% agree, the average isn't clear, or the average is out of reach;
  - a window shows each frame's spot (grey circles), the outliers (red X) and the final spot (green). **Confirm** [Enter] saves, **Rescan** [R] scans again, **Cancel** [Esc] saves nothing;
  - with no display (plain ssh) it asks in the terminal, and the picture is in `~/.table_place/overlay.png`. `--no-window` forces this.
- **`place` refuses a spot nobody confirmed** (`--no-confirm` overrides). The old `spot.json` is unconfirmed, so run `look` again first.
- **From Python:** `spot = choose_spot()` then `if spot: place_at(spot, execute=True)`.
- **Ready for other machines (Sheppy):** `arm_backend.py` is the one place that knows the arm stack. Paths and topics are environment variables (`TABLE_ARM_BACKEND`, `TABLE_CAMERA_NS`, `TABLE_URDF`, `TABLE_CALIB`, `TABLE_STATE_DIR`; see `README.md`).
- **Tests:** `test_spot_vote.py` 11/11 and `test_placement.py` 10/10 pass. **`test_place_bowl.py` was not run** (needs pybullet; Windows didn't have it).

## Next session, in order

### 0. Commit, then check on the robot machine
- [ ] Commit this work (`git status` shows the new and changed files). Also restore the script's executable bit: `git update-index --chmod=+x scan_and_detect.sh`.
- [ ] `python3 test_place_bowl.py`: must still pass (it wasn't run after today's changes).
- [ ] `python3 test_spot_vote.py --show`: try the window's buttons and keys with no arm.

### 1. Start the arm stack (rchi-cpu-5)
```bash
cd ~/feeding-deployment && export ARM_RPC_HOST=127.0.0.1
python3 src/feeding_deployment/control/robot_controller/arm_server.py   # terminal 1
python3 scripts/stub_base_server.py                                      # terminal 2
python3 scripts/bulldog_bypass.py                                        # terminal 3
python3 scripts/session/arm_set_speed.py low                             # once
```
With the bulldog bypass, the **physical e-stop is the only stop**.

### 2. Measurements still open
- [ ] **Measure from the wrist flange to the fingertips.** The model says 18.0 cm. Hand-measured camera-to-fingertip was 5.25 in, while the calibration and model put the lens about 22.8 cm behind the fingertip.
- [ ] **Check the table tilt.** The Sep 29 frame showed the table within 4° of the arm's level, which doesn't fit the 16° base tilt. `look` prints the tilt; check it after `level`.

### 3. Try the new `look` on the arm (gripper empty)
```bash
python3 scan_pose.py
python3 place_bowl.py look            # window: check the circles, then Confirm
```
- [ ] The grey circles should cluster on open table, and the green circle must not touch anything.
- [ ] Note the printed **spread** and how many of 10 agree. Run it 3–4 times.
  - Spread under 5 mm and nothing dropped every time → you can lower `--frames` to 5.
  - "frames disagree" with jumps of about 2 cm (each frame picks a slightly different, equally good spot) → raise `OUTLIER_TOL` in `spot_vote.py` to 0.025.
  - Real outliers (a frame far off) → `--frames 15`, or `--period 0.5`.
- [ ] The spot must be **0.75–0.95 m** from the arm base (`REACH_BAND`).
- [ ] Note how long the detection takes per frame (it's printed).

### 4. Place
```bash
python3 place_bowl.py place                       # plan only
python3 place_bowl.py place --execute             # empty gripper first
```
- [ ] Then with the bowl gripped by its lip (hand level, camera on the right seen from behind).
- [ ] **Can the camera see the table with the bowl in the gripper?** If not, always scan before gripping (as now), or filter out the bowl's points the way the gripper's are filtered (`SELF_RADIUS`).
- [ ] **Re-measure the reach range** for position-controlled lowering. 0.75–0.95 m came from the impedance-era planning; `python3 test_place_bowl.py --sweep` maps it.

## Sheppy / Jetson integration
How to connect is in the lab's "Connecting to Sheppy" notes: AnyDesk into Luxray, then `ssh jetson@192.168.55.1`. Keep the password out of this repo. Sheppy is run from `rammp-deployments/december_2026`. Decided: our scripts run **next to** Sheppy over ssh, not as a Sheppy service (for now).

- [ ] **Recon (read-only, nothing moves).** Paste the output to Claude:
```bash
cd ~/rammp-deployments/december_2026 && ls -la && git log --oneline -3
which sheppy; sheppy --help
cat *.yaml *.toml *.json 2>/dev/null | head -200
# with Sheppy running:
ros2 node list; ros2 topic list; ros2 action list; ros2 service list | head -80
ros2 topic echo --once /joint_states | head -30      # or whatever joint topic is listed
ls ~/.ros2/easy_handeye2/calibrations/
python3 -c "import numpy, scipy, yaml, cv2, pybullet; print('deps ok')"
pip3 list | grep -iE "kortex|feeding|moveit|pybullet"
ping -c 2 192.168.1.10                                # the arm
```
  This answers: how the arm is commanded there, the camera topic names, whether there's a hand-eye calibration for this arm, and whether the Python packages are installed.
- [ ] **Then (with Claude):**
  - if it isn't `feeding-deployment`'s `arm_server.py`, add a backend to `arm_backend.py` (its docstring lists the calls it needs);
  - export `TABLE_CAMERA_NS` etc. to match;
  - copy the code to the Jetson.
- [ ] On the Jetson: `python3 place_bowl.py level` again (the base tilt is per mounting), then steps 3–4 above.
- [ ] **IK:** `place_bowl.py` has its own IK (it starts from the current joints and keeps the bowl level). Only switch to the lab stack's IK if they want everyone on one package.
- [ ] Later: register `look` and `place` with Sheppy in `december_2026`, if wanted.

## If something goes wrong
- **Contact detected too early, or not at all:** tune `MOVE_TORQUE` (continuous lowering) or `STEP_TORQUE` / `TOTAL_TORQUE` (`--steps`) in `place_bowl.py`, using the printed torque changes.
- **`REFUSED: ... tool frame doesn't match`:** check the tool setting in the Kinova web app.
- **`STOPPED: reached the lowest planned height without touching`:** the bowl or table isn't where the plan thought. Nothing was released.
- **"may spin the long way" warning:** watch that joint, with your hand on the e-stop.
- **`--impedance`** is refused on this arm (tilted base, J6 assumption). Leave it off.

## Where things are
- Code: this repo (`RAMMP-table`). On the robot machine it lived in `~/feeding-deployment-table/table_placement` (branch `table-placement`, uncommitted) and, before that, in `~/walter_table` (old, don't edit). Pick one copy before the Jetson move.
- Saved state: `~/.table_place/` (`spot.json`, `obstacles.npy`, `mount.json`, `overlay.png`).
