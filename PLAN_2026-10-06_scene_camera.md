# Plan for 2026-10-06: scene camera, live placing areas, user picks the spot

New instructions from Swapnil (Slack, 2026-10-06). The previous plan is in `PLAN_2026-10-05.md`, which still applies to the arm-side placing.

## What changed
1. **The table scan happens after the fridge and microwave steps.** The arm is already holding the bowl when we look for a spot, so the wrist camera can't be used: the hand is roughly level, the camera points the wrong way, and the bowl and gripper block the view.
2. **A scene camera mounted behind the joystick will be used.** It's on the chair, like the arm. Swapnil's team is getting the transforms. **The rig will be at HERL to test on Thursday (2026-10-08).**
3. **The user picks the spot, not us.** Their UI (slide 10, "Placing: counter surfaces follow the same color rules") works like this:
   - while the chair drives with the arm holding an item, flat surfaces light up as the chair gets close;
   - **yellow** = out of reach or full; **blue** = a free stretch is in reach;
   - PLACE is yellow until the chair is "inside the halo", then blue; pressing it runs the placement.
   - Their "Engineering" note: *per surface: free or occupied, in reach or not, and the hotspot nearest the gripper.* The counter is a mapped room object (like the fridge), but **the system checks real-time surface conditions**. That real-time check is our part.

So our code changes from "find **one** spot, confirm it, place" to "**continuously report where the bowl can go**, then place at the spot the user picks".

## Is it possible?
Yes. Most of the existing code still applies:
- **`table_detect.py` stays almost unchanged.** It finds the plane, builds the free/occupied/unknown grid and works out the clearance for every cell. It already computes every cell where the bowl fits (`valid` in `_placement_search`). Today it throws away all but one.
- **`place_bowl.py place` / `place_at()` stays unchanged.** It already starts with the bowl gripped and takes a spot dict.
- **`table_detector_node.py`** already runs live at 2 Hz and publishes an overlay. This is the natural base for the live part.

What's new is the camera source, filtering the arm out of the view, the output format, and the live/parked split. Details below.

## Design

### 1. Camera pose from a fixed transform, not from the arm
- Today `choose_spot` computes the camera pose from the joint angles, the URDF and the hand-eye calibration (`model.fk(q, "end_effector_link") @ calib`).
- The scene camera and the arm base are both on the chair, so **camera → arm base is one fixed transform**. Load it from a file, or from TF if Swapnil's team publishes it.
- Add a "camera source" option (`TABLE_CAMERA=wrist|scene`), next to `TABLE_ARM_BACKEND`. Keep the wrist path working for the rig at the lab.
- The mount tilt still matters: everything must be expressed in the **level frame**. Run `place_bowl.py level` again on the HERL chair, because the tilt depends on how the arm is mounted.

### 2. Remove the arm and the held bowl from the scene camera's view
- Today only the gripper is filtered out (`self_distance`, `SELF_RADIUS`). From behind the joystick, the camera may see the **whole arm and the bowl**.
- Without a filter, the arm and bowl count as obstacles on the table, or as a second "table".
- **Fix:** model each arm link as a capsule (from the URDF with forward kinematics at the current joints), plus a cylinder for the bowl in the gripper. Drop depth points inside them before detection. This is the same idea as `self_distance`, extended to every link.

### 3. Report areas, not one spot
For each frame, publish what the UI needs, in the arm base / level frame:
- **state:** `no_table` | `out_of_reach` | `full` | `available`. This drives yellow vs blue.
- **free area:** where the bowl's centre can go (the `valid` cells), split into in-reach and out-of-reach. Send it as polygons, or as a small grid plus its origin and cell size. Ask the UI team which they want.
- **hotspot:** the valid, in-reach cell **nearest the gripper**. Today it's the cell nearest the middle of the camera view; nearest the gripper is a one-line change to the selection.
- **table height**, and how confident the result is (for example, how many recent frames agree).

### 4. Live while driving, steady when parked
- **While the chair drives:** run each frame on its own at about 2 Hz, with light smoothing. Frames can't be averaged because the table moves relative to the chair. This is enough to light surfaces up.
- **When the chair is parked and the user presses PLACE:** run the several-frame vote from `spot_vote.py` around the chosen point, then place. This keeps the noise protection from the 2026-10-05 work.

### 5. The user's choice comes back to us
- The UI sends the chosen point (in the base frame), or "use the hotspot".
- **Check it before moving:** it must be inside a valid cell (`grid_clearance` already does this), in reach (`REACH_BAND`), and the chair must not have moved since the scan.
- Then call `place_at(spot, execute=True)`. The `confirmed` flag now means "the user picked it in the UI". Our OpenCV Confirm window stays as a debug tool.

### 6. Re-tune for the new viewpoint
The thresholds were set for the wrist camera at about 0.5 m, looking down at the table. The scene camera is lower, farther away, and at a steeper angle:
- `TABLE_DIST` / `MAX_PLANE_DIST` (floor rejection) and the plane tilt limits;
- point density per cell: at a low angle the table is sampled sparsely, so the grid may need bigger cells;
- **shadows:** objects hide more of the table behind them from a low angle, and hidden cells count as blocked. This is safe, but it shrinks the free area;
- glare: shiny tabletops give depth holes. These also count as blocked, which is safe but shrinks the area.

## How big a change
**Medium.** The hard parts (detection, free-space grid, IK, placing, touch-sensed lowering) stay as they are. My estimate is roughly 400–600 new or changed lines, out of about 4000:

| Piece | Size | Notes |
|---|---|---|
| Fixed-transform camera source | small | Mostly plumbing in `choose_spot` / the node |
| Arm and bowl self-filter | medium | Capsules from URDF forward kinematics; needs a test on a real frame |
| Areas + state + hotspot output | medium | The data already exists in `_placement_search`; it needs packaging and a message for the UI |
| Live/parked split | small–medium | Builds on `table_detector_node.py` |
| User pick → check → `place_at` | small | Pieces exist |
| Tuning for the new view | unknown | Only knowable at HERL |

Most of the uncertainty is in the integration questions below, not in the code.

## Questions for Swapnil (before or on Thursday)
1. **The transform:** from the camera to which frame (arm base, chair `base_link`, or something else)? Published on TF or as a file? Is it the depth optical frame?
2. **The camera:** which model, and which topic names? Is depth aligned to colour available?
3. **The interface with the UI:** ROS 2 topic or service? Polygons, a grid, or pixels in the camera image? Should we propose a message?
4. **Does the UI send back a point** (in which frame), or just "PLACE" meaning "use the hotspot"?
5. **Who moves the arm for placing:** our `place_at` (our own IK and touch-sensed lowering) or their planner? Which arm backend runs on the HERL chair? (See `arm_backend.py`.)
6. **What is the arm's pose while driving with the bowl?** If it's fixed, the self-filter is easier, and we can check what the camera can see around it.
7. **Is the chair guaranteed stopped when PLACE is pressed?** Can we read odometry or the joystick to know?
8. **The "halo":** is it defined by our reach check, or by their map?

## Before Thursday (offline)
- [ ] Camera-source switch plus fixed-transform loading, with a placeholder transform.
- [ ] Areas/state/hotspot output from `_placement_search`, with tests in `test_placement.py`.
- [ ] A self-filter for the arm links and the held bowl, with a test in `test_place_bowl.py` (needs pybullet).
- [ ] Draft a message/JSON format for the UI and send it to Swapnil.

## At HERL on Thursday
- [ ] Record a few scene-camera frames (`save_frame.py`) with the table empty, cluttered, and with the arm holding the bowl in view.
- [ ] Check the transform: a point picked in the image should land on the right spot in the arm frame (touch it with the fingertip).
- [ ] `place_bowl.py level` on that chair.
- [ ] Re-tune the thresholds from section 6 on the recorded frames.
- [ ] Re-measure the reach with the bowl held (`test_place_bowl.py --sweep`). This sets when PLACE turns blue.
