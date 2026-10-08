# What to do next (updated 2026-10-08)

History: `PROGRESS_2026-09-29.md`, `PROGRESS_2026-10-05.md`, **`PROGRESS_2026-10-08.md`** (the HERL
session on Sheppy). Design: `PLAN_2026-10-06_scene_camera.md`.

## Next session: the two new pieces

### 1. Contact from the driver's end-effector force
The lowering has to stop **right as the bowl touches the table**. Everything tried on 2026-10-08
(where the hand stops, slowing down, falling behind the plan) was fooled by the impedance mode's
offset and drift. A local driver build (`~/v13-ws` on the Jetson) publishes an end-effector **wrench in
`/ee_state`**. When that is published by the running driver, use it:
- [ ] Check which driver is running and its message: `ps aux | grep kinova_gen3_node`;
      `source ~/v13-ws/install/setup.zsh && ros2 interface show rammp_arm_interfaces/msg/EeState`.
      Our side must source the **same** interfaces as the running driver.
- [ ] Note the wrench's field name and **frame** (base_link, or the tool frame), and what it reads at
      rest and how noisy it is (`ros2 topic echo /ee_state`).
- [ ] In `kinova_arm.py`, read the z force straight from `/ee_state` instead of `ee_force_z` (the
      estimate from the joint torques). Keep the rolling baseline and the "changes by more than
      `FORCE_RISE` for `FORCE_HITS` readings" test; keep the estimate as the fallback when the field
      is missing.
- [ ] Goal: down at 2 cm/s, **stops the moment it touches**, holds, setpoint = current joints, opens,
      backs out. Tune `FORCE_RISE` (5 N) from the printed numbers: empty gripper first, then the bowl.

### 2. The scene camera → arm transform, then placing at a target
Next time there will be the **transform matrices from the scene camera to the arm**, so the table, the
spot and the arm are all in one frame and `look` → `place` can put the bowl **at a target**, not just
straight down.
- [ ] Load them: either they're on TF (`ros2 run tf2_ros tf2_echo base_link
      scene_camera_depth_optical_frame`, which is what `look` reads now) or as a file (then add a
      loader next to the TF lookup in `place_bowl.grab_frames`).
- [ ] **Check them:** put a tape X on the table, find it in the camera image, and touch it with the
      fingertip (`place_bowl.py mark` prints where the fingertip is). They should agree within ~1 cm.
- [ ] Put clear table **0.70–1.05 m** from the arm base (the reach ring, `TABLE_REACH`), then
      `place_bowl.py look --no-window` → a confirmed spot.
- [ ] `place_bowl.py place` (plan only), then `--execute`: empty gripper, then the bowl.

## Before that (short)
- [ ] **Measure flange to fingertips** with a ruler (flange face to the ends of the closed fingers).
      The code assumes 17.955 cm (`TABLE_FLANGE_TO_TIP`); cuRobo calls 12 cm the "fingertip midpoint".
      This sets where the bowl centre lands.
- [ ] **`place_bowl.py level`** on Sheppy (the URDF has no mount tilt).
- [ ] **Run today's last version once** (`place --here --execute`, empty gripper) and keep the printed
      `z force … noise …` and `contact at …` lines. They're the baseline for step 1.
- [ ] Tell the driver team: switching into impedance, the arm **rises ~2.7 cm** (MEDIUM) or **sags
      ~3 cm** (SOFT), and drifts while moving. Probably the gravity / payload model.
- [ ] Commit today's work (nothing from 2026-10-08 is committed).

## Later
- `look` picks the spot nearest the middle of the camera view (a wrist-camera habit). Change it to
  the spot **nearest the gripper** (what the UI's "hotspot" means), and drop the "arm must be still"
  requirement for the scene camera.
- Filter the whole arm and a held bowl out of the scene camera's points (today only the gripper is).
- The live detector (`table_detector_node.py`) doesn't know the reach ring or the lip, so its green
  circle can be somewhere `place` can't go. Give it the same limits as `look`.
- With the bowl already gripped, `look` + `place` in one go (the scene camera isn't blocked by it).

## Setup on the Jetson
```bash
source ~/table_ws/install/setup.zsh        # interfaces matching the nightly driver (zsh!)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp TABLE_ARM_BACKEND=kinova TABLE_REACH="0.70,1.05"
cd ~/RAMMP-table
```
- Copy code from the laptop: `scp *.py abra@192.168.1.11:~/RAMMP-table/`.
- Live view: `ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8791`, then Foxglove on the
  laptop → `ws://192.168.1.11:8791`, Image panel `/table_detector/overlay` (detector running).
- `place` is a dry run unless `--execute`. A person on the e-stop for every `--execute`.

## If something goes wrong
- **`no /joint_states from kinova_gen3_node`:** the terminal lost its setup. Run the `source` and
  `export` lines again.
- **`go_to_ee_pose ended PLANNING_FAILED`:** cuRobo can't reach the pose (its goal is 12 cm in front of
  the flange, `CUROBO_TOOL_OFFSET`), or the target is below the arm base.
- **Contact in mid-air / too late:** the printed force change vs `FORCE_RISE` in `kinova_arm.py`.
  `--torque` uses the old torque sensing (`slide_down`) instead.
- **`look`: "outside the allowed area":** the clear table isn't 0.70–1.05 m from the arm base.
- **`ModuleNotFoundError` on the Jetson:** copy all the `.py` files, not only the changed ones.

## Where things are
- Code: this repo (`RAMMP-table`), copied to `~/RAMMP-table` on the Jetson. `sheppy.urdf` (the driver's
  URDF) lives there too. Saved state: `~/.table_place/`.
- The old rig (`rchi-cpu-5`, `TABLE_ARM_BACKEND=feeding`, wrist camera) still works the old way; its
  steps are in `PROGRESS_2026-10-05.md`.
