# What to do next (updated 2026-11-10)

**The placing lives in the lab's repo:** `capabilities/place_container` in
rammp-org/rammp-kitchen-capability-engines, PR #22 (issue #7), branch `feature/7-place-container` on
`WalterHsieh0713/rammp-kitchen-capability-engines` (last: `598cc97`, 2026-11-10).
**On 2026-11-10 it placed the container on flatbot, start to finish, ~2x faster than before**
(~30 s → ~15 s from carry to hand clear): one blended cuRobo carry down to 6 cm above the table, then
the slow impedance lowering, drop 1 cm, back out (or swing out). Details: `PROGRESS_2026-11-10.md`.

**Deadline (Cailyn): EOD Thursday 2026-11-12** — collision zones, a rosbag every run, smooth with
cuRobo (done), 10 runs from different starts with a success rate, UI integration (show every error).

## Running it on snorlax (`ssh snorlax@snorlax.local`)
- Clone in `~/place-container-dev`; don't touch `~/rammp-kitchen-capability-engines` (the lab's).
- **Work inside tmux:** `tmux new -s place` (after a drop: `tmux attach -t place`). SSH dropped 3x on
  Nov 10, each as the arm reset; snorlax's network stayed up, so it's the laptop Wi-Fi link. Try
  Ethernet.
- **Stack:** `sheppy up flatbot --manifest december_2026/sheppy-manifest.yaml` (the whole profile;
  not `flatbot_empty_world`, not single nodes: the driver needs `arm_state_publisher`, the planner
  the profile's chair/person volumes).
1. Update the code: `git pull` in `~/place-container-dev` (or `scp` changed files from the laptop),
   then `make build CAP=place_container`.
2. **Window 0, the node** (the defaults are the Nov 10 tuned values; no extra `-p` needed):
   ```bash
   docker run --rm -it --network host --ipc host -v $HOME/.place_container:/config/place_container \
     -v $HOME/cyclonedds.xml:/cfg/cyclonedds.xml:ro -e CYCLONEDDS_URI=file:///cfg/cyclonedds.xml \
     ghcr.io/rammp-org/rammp-kitchen-capability-engines/place_container:dev \
     ros2 run place_container place_container_node --ros-args -p use_orchestrator:=false -p allow_execute:=true
   ```
3. **Window 1 (Ctrl-b c), actions** after `source ~/RAMMP-table/env_snorlax.sh`:
   ```bash
   act() { ros2 action send_goal --feedback /place_container/execute_action \
     rammp_orchestrator_interfaces/action/ExecuteAction "{engine_id: place_container, action_id: $1, params_json: '${2:-{\}}'}"; }
   act mark                                   # fingertips on the spot, teleop off
   act reset '{"execute": true, "empty": true}'
   act place '{"execute": false}'             # dry run, container in the hand
   act place '{"execute": true}'
   act reset '{"execute": true, "empty": true}'
   ```
   Spots **~0.88–0.90 m** from the base (0.81–0.83 is borderline). A mark expires after 30 min.
   **Don't use `allow_reconfigure`** on flatbot: the other posture's elbow goes into the person's lap.
4. **If every goal is rejected:** `docker logs --tail 60 sheppy-arm 2>&1`. `ArmLink: faulted` →
   `ros2 service call /clear_faults std_srvs/srv/Trigger`. Another owner → `/control_status`.
5. **Recover a different arm posture** (empty hand): `act reset '{"execute": true, "empty": true, "big_turn": true}'`.
   Reset refused inside the chair's padding: restart the node with `-p reset_mode:=legacy` once.

## Checklist to EOD Thursday
**Wednesday (no robot):**
1. [ ] **A rosbag every run inside the capability** (as the fridge code; Cailyn's link): arm state,
       gripper, the node's log, action feedback, errors. Today only the descent CSV and
       `tools/run_trials.py` (untracked) record.
2. [~] **UI integration** — code done Nov 10 (`c1db1e7`, on top of #42 `feat/engine_update` merged into
       the branch; merge `dev` once #42 lands): a panel with each phase + progress bar, every error
       with an OK button (waits ≤ `ui_error_wait_s`), mock walk-through + `mock_fail` for a sim.
   - [ ] **Ask Guo how to run the UI bridge** next to the orchestrator (the README doesn't say).
   - [ ] **Try it in mock** with the orchestrator (`make run CAP=place_container MOCK=1`, not
         `STANDALONE`, so it gets a session token): `place` (phases), then
         `place '{"mock_fail": "lower"}'` (the error panel; press OK).
   - [ ] Then on flatbot with the orchestrator + UI (today's runs were standalone: no panel).
3. [ ] **Tests on snorlax's Docker:** `make test-arm ARM_CMD='python3 -m pytest -q -p no:cacheprovider capabilities/place_container/test'`,
       `make check`, `make lint`. The Nov 10 tests (swing, back-off, approach split, arming) have
       never run.
4. [ ] **Ask Swap** whether flatbot's planner volumes are the "new collision zones".
5. [ ] **Make the backing-away smooth: one blended cuRobo motion from letting go back to the start.**
       Cailyn (Slack, Nov 10): "it doesn't look particularly smooth backing away". I'd told her the
       place uses cuRobo blending; that's true only for the carry/approach and the reset.
   - **Now** (`node.py` `_place` after contact, `lower.py`): hold (`hold_goal` + `wait_still`) →
     `let_go` (waits for the gripper ≤ 0.3) → `time.sleep(0.5)` → `sink` (1 cm drop: `hold_goal` +
     `wait_still`) → `back_out` (`move_joints` in impedance through `plan_retreat`'s waypoints:
     straight back, then the 60° swing, a sharp unblended corner between) → `wait_still` → place ends
     in impedance. `reset` is a separate action (`exit_impedance`, then `go_to_joint_configs` up →
     start). So: several stops, a corner, and a gap before the reset.
   - **Plan:** after the gripper is open, ONE `go_to_joint_configs` (position mode, cuRobo, blended)
     through: the drop's joints (`sink`'s `plan_descent` end) → `plan_retreat`'s waypoints, thinned
     to every ~1–2 cm (straight part + swing) → straight up (`reset_moves`' "up") → the start joints
     (`start.json`). Small blend radii near the container (~0.01 m; the table and container aren't
     in cuRobo's world), `blend_radius_far` once up high, 0 at the start. The same trick as the
     carry/approach (`_carry_joints`). The place then ends back at the start: no separate reset.
   - Leave impedance first, without a jump: `exit_impedance` (position setpoint = current joints)
     after the gripper opens; drop the 0.5 s sleep and the `wait_still`s in between.
   - Keep the old path behind a parameter (e.g. `return_mode: blended | legacy`) for the first
     test; the dry run checks the new legs with `_check_carry_joints`-style cuRobo plans.
   - The PR should say why the lowering stays `move_joints` (it must stop on contact; AGENTS.md:
     "when you keep a legacy move, say why").
   - **Test Thursday, attended, before the 10 runs:** do the fingers leave the lip cleanly in
     position mode (no drag), does the container stay put, any corner cut toward the container.

**Thursday at the lab (~3 h; book 4–5):**
1. [ ] Bring up, build, tests; tmux (Ethernet if possible).
2. [ ] Mark at ~0.88–0.90 m.
3. [ ] **10 runs from different start poses, a bag every run.** On a failure: note the error and the
       bag, keep going. Copy every bag and `runs.csv` off the robot before leaving.
4. [ ] UI check: force one error (e.g. a spot too close) and see it on the UI.
5. [ ] Success rate + timings (`runs.csv` `duration_s`) → push, update PR #22, mark it ready.

### Still open from Oct 29
- [x] Gripper: worked all of Nov 10.
- [x] Descent recording review: the hand rises only 0.18 cm after the contact cancel; the settle is
      gone (`settle_s` 0).
- [ ] Tell the lab: the scene camera's transform is ~12° off in yaw; the arm sees the table 6.4°
      tilted (check with a level / flatbot's `mount.yaml`).
- [ ] Tell Swapnil the example Dockerfile lacks `rosdep update`.

History: `PROGRESS_2026-09-29.md`, `PROGRESS_2026-10-05.md`, `PROGRESS_2026-10-08.md`,
`PROGRESS_2026-10-19.md` (impedance placing on the HERL chair works, `--here`),
`PROGRESS_2026-10-23.md` (choose the spot by clicking), `PROGRESS_2026-10-26.md` (flatbot / snorlax:
placing at a marked spot works; setup, contact threshold, scene camera depth),
`PROGRESS_2026-10-29.md` (place_container in the lab's repo works on flatbot; camera detection
settings, `look --no-check`, the camera transform check, the gripper failure),
**`PROGRESS_2026-11-10.md`** (fast, smooth place on flatbot; six fixes from the arm; SSH drops).
Design:
`PLAN_2026-10-06_scene_camera.md`.

Today's robot is **flatbot** (arm on a 30° mount), run from **snorlax**. Setup:
`PROGRESS_2026-10-26.md`, section 1. The HERL chair setup further down is the Oct 19–23 one.

## Next session: the scene camera chooses the spot
`mark` → `place --execute` puts the bowl on a sticky note on flatbot. Next: let `look` choose the
spot, then `place` there.

1. [ ] **Depth on the plain table** (it's streaky; see the Oct 26 notes, section 5):
   - lay a printed page flat on the table and watch `/scene_camera/depth/image_rect_raw` in Foxglove;
   - if the page is solid and the table isn't, it's texture: put a patterned mat under the placing
     area, or try a stronger dot projector;
   - if the page is streaky too: the camera is only ~14 cm above the table; ask the lab about mounting
     it higher or tilting it down (~30° or more onto the table).
2. [ ] **First, rerun `py place_bowl.py look --no-window`.** At the end of Oct 26 it saw the table
       34° from level in every frame ("wrong plane?"), where earlier the same day it read ~0°. The
       camera's TF didn't change. Was the camera moved/tilted? Then its mount angle in the lab's
       `static_tf` config is out of date. Otherwise check `overlay.png` for which plane it picked.
3. [ ] **Is the X in view?** Put the tape X where the camera sees open table (the 3D panel in
       `arm_mount` shows where it looks). The Oct 26 X (0.764, −0.248) was outside the image.
4. [ ] **Check the camera transform in x/y:**
   - `mark` the X, then run
     `look --no-window` (flatbot's depth topic and range are the defaults now);
   - type `at X Y` (the marked point), then copy `~/.table_place/overlay.png` and check the red X / circle sits
     on the tape;
   - with a cup on the X, it should also show as an object there. Within ~1 cm = good.
   - Height already agrees (0.19 vs 0.198 m).
5. [ ] **`look` picks the spot:**
   - its required 13.5 cm from every visible table edge didn't fit the narrow strip it sees; this may
     need the camera fix, or a smaller margin for the scene camera;
   - then Confirm, and `place --execute`.
6. [ ] **Make the code match flatbot:**
   - `TABLE_VZ_STOP=-0.01` as the default;
   - `test_place_bowl.py` for the kinova backend.

## Also open
- The bump at the end of the retreat (z +1 cm, ~27 N) when the arm leaves impedance
  (`descent_20261026_125221`).
- Flange to fingertips: still the 17.955 cm default; never measured with a ruler.
- `depth_registration` on the scene camera: the overlay is drawn on colour but the depth may not line up
  with it.

## Earlier plan (HERL chair, Oct 23): put the bowl down at a chosen spot
Done on flatbot on Oct 26 (`mark` → `place`). The checklist below is kept for the HERL chair.

Most of it exists from the old rig; it hasn't been run on the HERL chair or with today's driver:
- `place --at X Y [--table-z Z]`: bowl centre at (X, Y) m, level frame (= `arm_mount` on the HERL chair:
  origin on the arm base axis, x forward, y left).
- `mark`: touch the target with the fingertip; it saves that point (and the table height) as the
  spot. Then `place` with no arguments goes there.
- `look`: the scene camera suggests a clear spot; **click the table to choose another** (or type
  `at X Y` at the terminal prompt). It checks the bowl fits there in most frames and is in
  reach, then Confirm saves it. Not tried on the robot yet.

The steps `place` takes to a spot: adjust → lift to `SAFE_HEIGHT` (30 cm above the table) →
transit (go_to_ee_pose, `HOLD_LEVEL`) to above the spot → the impedance lowering from there →
open, retreat.

### Before (short)
- [ ] **Measure flange to fingertips** with a ruler (flange face to the ends of the closed
      fingers). The code still uses the old rig's 17.955 cm (`TABLE_FLANGE_TO_TIP`). It sets where
      the bowl centre lands: `HOLD_REACH` = bowl radius + this.
- [ ] **Re-sweep the reach** on the new model: `python3 test_place_bowl.py --sweep <table z>` on
      the Jetson. The 0.70–1.10 m from Oct 8 was with the old URDF. Then
      `export TABLE_REACH="min,max"`.
- [ ] **Table height:** `python3 place_bowl.py mark` with the fingertip on the table prints it (z,
      level frame). `--at` needs it (`--table-z`, or the last saved spot).

### Then
1. [ ] **Plan only:** `python3 place_bowl.py place --at X Y --table-z Z`. Pick X, Y inside the
       reach ring, about 0.8 m from the base. Check the steps it prints: lift, transit, down, and
       their heights.
2. [ ] **Things likely to need fixing on the way** (they changed today, the spot path wasn't run):
   - The transit is `go_to_ee_pose` with **`HOLD_LEVEL`**. The hand now points **5° down**
     (`PLACE_PITCH_DEG`). Will the driver's "level" undo that? If so, `place` stops with "bowl is
     … deg off its hold after the moves". Then use `HOLD_FIXED` for the transit, or our own joint
     moves.
   - The lift and transit are **position** moves, while the arm may still be in impedance from
     the last run. Watch the first move.
   - The descent starts at `SAFE_HEIGHT` (30 cm). At 2 cm/s that's ~15 s; maybe hover lower first
     (`HOVER_GAP`).
   - The `EARLY_STOP` check (contact more than 3 cm above where the bowl should meet the table =
     refuse) only works with a known table height. That's the one protection against a false
     vz contact in mid-air letting go of the bowl.
3. [ ] **Execute with the empty gripper** (`--execute`), then `plot_descent.py`. Check it lands
       where you said: `mark` the fingertip there, or measure.
4. [ ] **With the bowl.** The weight changes the impedance sag and the vz signal: compare the
       plots.
5. [ ] **Choosing the spot:** typed coordinates (`--at`) first. Then `look`'s camera pick, once the
       scene camera → `arm_mount` transform is right: check it with a tape X on the table, the
       camera's pick vs `mark`, within ~1 cm.

## Later
- Review today's last recordings (velocity contact, planned retreat) in
  `~/.table_place/descents/` on the Jetson: the retreat's `z` should no longer rise.
- Tell the driver team:
  - the `/ee_state` "external" force drifts ~10 N with the pose (empty gripper, still);
  - `arm_robotiq_85_left_inner_knuckle_joint` in `/arm/robot_description` doesn't load in pybullet;
  - the HERL chair's `mount.yaml` is level (identity): is that right?
- `look` picks the spot nearest the middle of the camera view (a wrist-camera habit). Change it to
  the spot **nearest the gripper**, and drop the "arm must be still" requirement for the scene camera.
- Filter the whole arm and a held bowl out of the scene camera's points (today only the gripper is).
- The live detector doesn't know the reach ring or the lip. Give it the same limits as `look`.

## Setup on the Jetson
- **Stack:** `sheppy` (the lab's launcher for the ROS nodes) in `~/rammp-deployments/december_2026/`. Start `arm_state_publisher`, `arm`
  and `planner`, all nightly, the state publisher first. Not `home_arm` or the demos.
- **Each terminal:**
  ```bash
  source ~/table_ws/install/setup.zsh        # interfaces matching the nightly driver (zsh!)
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp TABLE_ARM_BACKEND=kinova TABLE_REACH="0.70,1.05"
  cd ~/RAMMP-table
  ```
- **After a driver update:** `python3 save_urdf.py`. If the interfaces changed, pull
  `~/table_ws/src/rammp-interfaces-ros2` (dev) and run
  `colcon build --packages-up-to rammp_arm_interfaces`.
- **Copy code from the laptop:** `scp *.py abra@192.168.1.11:~/RAMMP-table/`. The laptop is on the
  USB Ethernet adapter, 192.168.1.111.
- `place` is a dry run unless `--execute`. A person on the e-stop for every `--execute`.
- **Debug loop:** `reset --save` once, then `place --here --execute` → `plot_descent.py` →
  `reset --execute`.

## If something goes wrong
- **`no /joint_states from kinova_gen3_node`:** the terminal lost its setup, or the stack is down
  (`docker ps`). Run the `source` / `export` lines again.
- **`Cannot load URDF` / names don't match:** `python3 save_urdf.py` (the driver was updated).
- **`go_to_ee_pose ended PLANNING_FAILED`:** cuRobo can't reach the pose. Its goal is 12 cm in front
  of the flange, `CUROBO_TOOL_OFFSET`.
- **Contact in mid-air / too late:**
  - plot the run (`plot_descent.py`);
  - `TABLE_VZ_STOP=-0.005` stops sooner;
  - `TABLE_CONTACT=force` uses the force tests (`TABLE_FORCE_RISE_Z`, `TABLE_FORCE_RISE`).
- **It presses too hard / springs up while opening:** `TABLE_HOLD_PRESS` (m, default 0.003).
- **The hand stopped with the gripper open:** `python3 place_bowl.py retreat --execute`, then
  `reset --execute`.
- **`ModuleNotFoundError` on the Jetson:** copy all the `.py` files, not only the changed ones.
- **zsh:** quote patterns: `grep '[k]inova'`, not `grep [k]inova`.

## Where things are
- **Code:** this repo (`RAMMP-table`), copied to `~/RAMMP-table` on the Jetson. `sheppy.urdf` (from
  `save_urdf.py`) lives there too. Saved state is in `~/.table_place/`: spot, start pose,
  `descents/`.
- **The old rig** (`rchi-cpu-5`, `TABLE_ARM_BACKEND=feeding`, wrist camera) still works the old
  way; its steps are in `PROGRESS_2026-10-05.md`.
