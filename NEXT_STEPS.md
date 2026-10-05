# Where I left off (2026-10-04, late)

**Change after the first arm test:** the lab's compliant (impedance) mode dropped the wrist into
the table. Its gravity model assumes J6 = -67.6 deg; ours was at +60 deg. `place` no longer uses
compliant mode. It now lowers the last few cm in 2 mm position-controlled steps and stops when
the joint torques jump (touch sensing). It refuses ever to enter compliant mode.

## Next (planned, not coded yet)
See `PLAN_2026-10-05.md`: scan several frames, average the green circles (dropping outliers), click Confirm on the camera picture, then place there.

## Where things are
- **Code:** `~/feeding-deployment-table/table_placement/` (this folder).
  - `~/feeding-deployment-table` is a separate copy of the `feeding-deployment` repo (a git worktree), on branch `table-placement`. Nothing is committed yet.
  - `~/walter_table` is the old copy. Don't edit it any more.
- **Main script:** `place_bowl.py`. It has two commands:
  - `look`: find a free spot on the table and save it;
  - `place`: put the held bowl down on that spot.

  The full explanation is in the comment at the top of the file.
- **Saved spot and overlay picture:** `~/.table_place/` (`spot.json`, `overlay.png`).
- **To reopen the Claude chat:** the Claude Code panel in VS Code (past conversations), or `claude --resume` in a terminal.

## Step 1: allow real-time priority (done, no longer needed by `place`)
```bash
echo "dhyi - rtprio 99" | sudo tee /etc/security/limits.d/99-realtime.conf   # your Linux password
```
Log out of the desktop and back in, or reboot. Then `ulimit -r` should print `99`. Start the arm stack from terminals opened after logging back in.

## Step 2: start the arm stack
```bash
cd ~/feeding-deployment && export ARM_RPC_HOST=127.0.0.1
python3 src/feeding_deployment/control/robot_controller/arm_server.py   # terminal 1
python3 scripts/stub_base_server.py                                      # terminal 2
python3 scripts/bulldog_bypass.py                                        # terminal 3
python3 scripts/session/arm_set_speed.py low                             # once
```
With the bulldog bypass, the **physical e-stop is the only stop**.

## Step 3: checks before the first real place
1. **Measure from the wrist flange to the fingertips.** The model says 18.0 cm. If it's different, tell Claude.
2. **Test the touch-down with no bowl:**
   - close the gripper on nothing;
   - put the hand level with the camera on the right, a few cm above the table;
   - then, from this folder:
   ```bash
   python3 place_bowl.py place --here              # plan only, nothing moves
   python3 place_bowl.py place --here --execute    # 2 mm steps down, prints the torque change per step
   ```
   - First try it in mid-air, about 15 cm up. It should end with `STOPPED: reached the lowest planned height`. The torque numbers it prints are the noise level; send them to Claude.
   - Then try it about 4 cm above the table, with `--max-drop 0.07`. It should print `contact at tool z=...`. Press Ctrl-C at the open-gripper prompt.
3. **Check `look`:**
   - empty the gripper and run `python3 scan_pose.py`, then `python3 place_bowl.py look`;
   - open `~/.table_place/overlay.png`: the green circle should sit on open table;
   - the printed spot must be **0.75–0.95 m** from the arm base.

## The arm base is mounted ~16 deg tilted
- Hold the hand truly level, checked with a phone level, with the camera on the right. Then run `python3 place_bowl.py level` once. It saves true "up" to `~/.table_place/mount.json`, and `look` and `place` then use it for level and straight down.
- Run `look` again afterwards. A spot found before `level` is refused.
- `--impedance` is refused on a tilted base: the lab's compliant controller assumes an upright base for gravity.

## Optional: impedance (`--impedance`; refused while the base is tilted)
- It only works with J6 at -67.6 deg, which here means the elbow flipped (J4 about +90). From the usual posture the switch is about a 175 deg move, so the script refuses.
- To test it without the bowl, first jog the arm to this pose. In the web app, 0-360 form: **352.0, 109.2, 120.5, 89.5, 350.6, 292.4, 59.0**. The hand ends up about 15 cm above the table.
- Then run `python3 place_bowl.py place --here --impedance` (plan only), and add `--execute`.

## Step 4: the real place
Grip the bowl by its lip, with the hand level and the camera on the right seen from behind. Then:
```bash
python3 place_bowl.py place              # plan only
python3 place_bowl.py place --execute    # [ENTER] before moving and before letting go
```

## If something goes wrong
- **Contact detected too early, or not at all:** tune `STEP_TORQUE` / `TOTAL_TORQUE` in `place_bowl.py` from the printed torque changes.
- **`REFUSED: ... tool frame doesn't match`:** check the tool setting in the Kinova web app.
- **`STOPPED: reached the lowest planned height without touching`:** the bowl or table isn't where the plan thought. Nothing was released.
- **"may spin the long way" warning:** watch that joint, with your hand on the e-stop.

## Numbers in use
| What | Value | Where |
|---|---|---|
| Bowl radius (body) | 5.5 cm | `table_detect.py` `BOWL_RADIUS` |
| Lip width | 1.25 in | `place_bowl.py` `LIP_WIDTH` |
| Lip to bowl bottom | 8 cm | `BOWL_DEPTH`, or `--bowl-depth` |
| Tool frame to fingertip | 5.96 cm | from the arm model |
| Hover (bowl bottom above table) | 10 cm | `HOVER_GAP` |
| Touch sensing starts at | 3 cm above table | `PRE_GAP` |
| Touch step / contact torque | 2 mm / 1 Nm per step, 2 Nm total | `PATH_STEP`, `STEP_TORQUE`, `TOTAL_TORQUE` |
| Back off after release | 7.9 cm (2.5 × lip) | `RETREAT` |
| Table height | 0.135 m above arm base | measured from the saved frame |
| Reachable spots | 0.75–0.95 m from arm base | `REACH_BAND` |

## Open questions
- You measured 5.25 in from camera to fingertip. The calibration and model say about 22.8 cm, and the saved depth frame shows the nearest part of the fingers at 16.9 cm. Measuring from the flange to the fingertips (step 3.1) settles it.
- This assumes the fingertips are pushed in all the way to the bowl wall when gripping. If they aren't, the bowl lands short of the circle center by that amount.
