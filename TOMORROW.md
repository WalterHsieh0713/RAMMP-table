# Table detection — robot checklist

Files in this folder:
- `table_detect.py` — the detector (works on saved images; run `python table_detect.py` at home to see the demo)
- `table_detector_node.py` — ROS 2 node that runs the detector live on the wrist camera
- `save_frame.py` — grabs one frame from the robot camera so you can test offline
- `scan_pose.py` — **moves the arm** to the fixed table-scan pose (see "Scan pose" below)
- `test_placement.py` — offline tests for the bowl-placement search (`python3 test_placement.py`, no camera needed)
- `table_overlay.png` — what a working result looks like (red = table, yellow = stuff on it, green circle = where a bowl fits)

Only needs `numpy` and `scipy` on the Jetson. Open3D is optional.

**New: it also finds where a bowl can go.** On top of the red table you now get
yellow for anything standing on it and a green circle on the biggest clear patch
nearest the middle of the view. See the top of `table_detect.py` for how it works.
Three new settings, same names everywhere:

| what | `table_detect.py` flag | `table_detector_node.py` parameter | default |
| --- | --- | --- | --- |
| bowl radius (m) | `--bowl-radius` | `-p bowl_radius:=` | `0.075` — **placeholder, set the real bowl** |
| extra clearance around the bowl (m) | `--margin` | `-p margin:=` | `0.02` |
| top-down grid resolution (m) | `--cell-size` | `-p cell_size:=` | `0.005` |

```
python3 table_detect.py --color robot_color.png --depth robot_depth.png     --fx 615 --fy 615 --cx 320 --cy 240 --depth-scale 1000 --largest-plane     --bowl-radius 0.075 --out robot_overlay.png
python3 table_detector_node.py --ros-args -p bowl_radius:=0.075 -p margin:=0.02
```

**Safety:** don't command the arm to move unless someone is in the lab with a hand on the e-stop. `scan_pose.py` (below) **moves the arm**; everything else here is perception-only and never commands motion.

## Scan pose — move the arm before detecting (this MOVES the arm)

`scan_pose.py` sends the Kinova to one fixed joint pose where the wrist camera sees the whole
table, then exits. It is separate from the detector on purpose: run it, then start the node.
Change the pose by editing `SCAN_POSE` (radians, J1..J7) at the top of the file.

On this machine (`rchi-cpu-5`, arm plugged in directly, no NUC) the arm stack isn't
started by anything else. Bring it up in three terminals and leave them running.
`bulldog_bypass.py` unlocks motion and gives you **no software e-stop**: the physical e-stop is
the only stop.

```
cd ~/feeding-deployment
export ARM_RPC_HOST=127.0.0.1                                  # in each of the 3 terminals
python3 src/feeding_deployment/control/robot_controller/arm_server.py    # terminal 1
python3 scripts/stub_base_server.py                                       # terminal 2
python3 scripts/bulldog_bypass.py                                         # terminal 3
```

Then, from `~/walter_table` (`scan_pose.py` already defaults to `127.0.0.1`):

```
python3 ~/feeding-deployment/scripts/session/arm_set_speed.py low    # once per session (needs ARM_RPC_HOST=127.0.0.1)
python3 scan_pose.py --dry-run     # reads the arm, prints current/target/delta, sends nothing
python3 scan_pose.py               # same, then [ENTER] to move; checks it arrived within 1 deg
python3 table_detector_node.py     # then detect as usual (camera topics already match here)
```

**One terminal instead (for recording):** `./scan_and_detect.sh` runs `scan_pose.py`, and if the
arm arrives it opens the overlay viewer (`rqt_image_view`) and starts `table_detector_node.py` in the same terminal; Ctrl-C closes both. `--yes` skips the [ENTER]
prompt, and anything after that is passed to the node:
`./scan_and_detect.sh --yes --ros-args -p bowl_radius:=0.075`. Still needs `export ARM_RPC_HOST=127.0.0.1`
and the speed set to `low` first.

It refuses to move if bulldog_bypass isn't running, the speed isn't `low`, the gripper is
closed, or a joint would travel more than 150 deg (`--max-jump` to change).
Shut down in reverse: Ctrl-C bulldog_bypass → arm_server → stub_base_server.
There is no arm-to-camera TF on this machine, so the detector logs "No TF" and uses the
largest-plane fallback. That's expected.
If it prints `WARNING: Jn ... the long way`, watch that joint on the first run.

---

## Morning — get the camera data (goal: a red table on a real robot frame)

1. **Connect:** AnyDesk → Jetson ID from the lab sheet. Open a terminal.
2. **Which ROS?**
   - `echo $ROS_DISTRO` → `humble` (or similar) means ROS 2. Good, the scripts are ROS 2.
   - If `ros2` isn't found but `rostopic list` works, it's ROS 1 — stop and ask Claude to convert the node (10 min).
3. **Is the camera running?** `ros2 topic list | grep -i camera`
   - Nothing? Start it with the RAMMP bringup command (from the Demo-Software repo; it waits ~8 s before the camera comes up):
     ```
     ros2 launch rammp_prototype_bringup camera.launch.py params_file:=$(ros2 pkg prefix rammp_prototype_bringup --share)/config/camera_wrist.yaml
     ```
   - "Package not found"? Source the workspace first (`source ~/ros2_ws/install/setup.bash` or wherever Demo-Software lives), or try `sheppy`.
   - Still stuck? Ask your PhD student how they launch the wrist camera.
4. **Check the depth stream** (the scripts use the depth image aligned to color by default):
   - `ros2 topic hz /camera/wrist/aligned_depth_to_color/image_raw` (should be ~15 Hz)
   - `ros2 topic echo --once /camera/wrist/aligned_depth_to_color/camera_info` → note `frame_id` and `k`
   - If your topic names differ, pass them with the `--ros-args -p depth_topic:=... -p info_topic:=... -p color_topic:=...` flags below.
   - **If the camera is an OAK-D (Luxonis)** — check with `lsusb` (shows "Luxonis"/"Movidius"): the driver is `depthai_ros_driver` (`ros2 launch depthai_ros_driver camera.launch.py`) and topics are usually `/oak/stereo/image_raw` (depth), `/oak/stereo/camera_info`, `/oak/rgb/image_raw`. Confirm with `ros2 topic list`, then use:
     ```
     --ros-args -p depth_topic:=/oak/stereo/image_raw -p info_topic:=/oak/stereo/camera_info -p color_topic:=/oak/rgb/image_raw
     ```
     OAK-D notes: keep the camera ~40 cm+ above the table (stereo can't see closer than ~20–35 cm), and plain textureless tabletops can give patchy depth — tilt the camera, add light, or put a placemat down.
5. **Copy this folder to the Jetson** (AnyDesk's file transfer), e.g. to `~/walter_table/`, then `cd ~/walter_table`.
6. **Dependencies:** `python3 -c "import numpy, scipy"` — if that errors, `pip3 install scipy`.
7. **Offline test on a real frame:**
   - `python3 save_frame.py` (add `--ros-args -p depth_topic:=... -p info_topic:=...` if names differ)
   - Run the command it prints → open `robot_overlay.png`. Red should be on the table.
   - Add `--bowl-radius 0.075` (or the real bowl radius) to see the green placement circle too.
   - It also prints `Bowl placement (camera frame): x=... y=... z=... m`. If it says "No room for the bowl", the message says why.

## Night — live demo + video

8. **Run live:** `python3 table_detector_node.py` (same `--ros-args` if needed)
   - Log line to look for: `Table at x=... y=... m, surface height z=... m in base_link`
   - And right after it: `Bowl spot at x=... y=... z=... m in base_link, radius ... cm, ... cm clear` — that is the point to hand the arm.
   - A warning about "No TF" is OK — it falls back to "largest plane", which works when the camera is aimed at the table.
9. **Watch it:** new terminal → `ros2 run rqt_image_view rqt_image_view /table_detector/overlay`
10. **3D view (optional, looks great in the video):** `rviz2` → Fixed Frame = `base_link` (or the camera frame_id if no TF) → Add → Marker `/table_detector/marker` (+ Image `/table_detector/overlay`)
    - Two markers share that topic: the red slab is the table (id 0), the green disc is the bowl spot (id 1).
11. **Record:** screen-record the overlay/RViz on your laptop (Windows: `Win+Alt+R`, or AnyDesk's session recording) + a phone video of the arm/camera looking at the table.

## If something's off
- **Overlay has no red:** camera too close (<10 cm) or too far (>3 m) from the table, or pointed away.
- **Wall or floor turns red:** no TF + table isn't the biggest surface in view — aim the camera more at the table.
- **Red is patchy / full of holes (OAK-D):** stereo depth struggles on plain surfaces — more light, a textured placemat, or a different angle.
- **Image looks upside down:** the wrist camera is mounted upside down. Detection doesn't care.
- **No green circle, "no room for the bowl":** read the rest of that message. If it ends with "only N depth points per 5 mm cell", the camera is too far away for a 5 mm grid — pass `--cell-size 0.01` (or `-p cell_size:=0.01`). Otherwise the table really is too cluttered, or the bowl radius is too big — clear a space and try again.
- **Circle sits somewhere silly:** anything the camera can't see counts as blocked, so the shadow behind a tall object is off limits on purpose. Move the camera so it looks down more.
- **Everything else:** paste the terminal error to Claude.
