# What to do next (updated 2026-10-26)

History: `PROGRESS_2026-09-29.md`, `PROGRESS_2026-10-05.md`, `PROGRESS_2026-10-08.md`,
`PROGRESS_2026-10-19.md` (impedance placing on the HERL chair works, `--here`),
`PROGRESS_2026-10-23.md` (choose the spot by clicking), **`PROGRESS_2026-10-26.md`** (flatbot / snorlax:
placing at a marked spot works; setup, contact threshold, scene camera depth). Design:
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
2. [ ] **Is the X in view?** Put the tape X where the camera sees open table (the 3D panel in
       `arm_mount` shows where it looks). The Oct 26 X (0.764, −0.248) was outside the image.
3. [ ] **Check the camera transform in x/y:**
   - `mark` the X, then run
     `look --no-window --max-plane-dist 1.0 --depth-topic /scene_camera/depth/image_rect_raw`;
   - type `at X Y` (the marked point), then copy `~/.table_place/overlay.png` and check the red X / circle sits
     on the tape;
   - with a cup on the X, it should also show as an object there. Within ~1 cm = good.
   - Height already agrees (0.19 vs 0.198 m).
4. [ ] **`look` picks the spot:**
   - its required 13.5 cm from every visible table edge didn't fit the narrow strip it sees; this may
     need the camera fix, or a smaller margin for the scene camera;
   - then Confirm, and `place --execute`.
5. [ ] **Make the code match flatbot:**
   - `TABLE_VZ_STOP=-0.01` as the default;
   - the scene camera's depth topic `image_rect_raw` and `max_plane_dist` 1.0 as the defaults (or per
     robot);
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
