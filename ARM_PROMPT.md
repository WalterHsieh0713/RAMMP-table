# Claude Code prompt: move the arm to a fixed scan pose before detecting

Run this in the folder containing table_detect.py, table_detector_node.py,
save_frame.py and TOMORROW.md (probably ~/walter_table). Model: Opus 5, high effort.
First: `git add -A && git commit -m "before arm pose"` so I can undo.

---

## Context

This folder is a PERCEPTION-ONLY ROS 2 project. table_detect.py finds a table in a
depth image and finds a clear circle where a bowl can be set down.
table_detector_node.py runs it live on the wrist camera at 2 Hz and publishes
/table_detector/overlay and /table_detector/marker. Nothing here moves the arm.

The robot is a Kinova Gen3 7-DOF arm. I found a viewing angle where the camera sees
the whole table, and I want the arm to go to that pose before scanning.

## Step 1: read the existing arm code (do not search the whole machine)

The arm code is in ~/feeding-deployment. Read these, and only these, to start:

    ~/feeding-deployment/README.md
    ~/feeding-deployment/src/feeding_deployment/robot_controller/arm_server.py
    ~/feeding-deployment/src/feeding_deployment/robot_controller/kinova.py

Then find the preset-pose scripts, which are the closest thing to what I want. The
README calls them retract.py / transfer.py / acquisition.py, reached by an alias
`cd_actions`:

    find ~/feeding-deployment -name "retract.py" -o -name "acquisition.py"
    grep -n "cd_actions\|launch_arm" ~/.bashrc ~/.bash_aliases 2>/dev/null

READ-ONLY in ~/feeding-deployment. Never modify, move or delete anything there.

Establish, with evidence from real code rather than assumption:
  1. How a preset joint configuration is actually sent (the exact call in retract.py).
  2. **Degrees or radians.** This is the one that will hurt if it's wrong. Copy the
     convention from a call that already works.
  3. Joint order and any wrapping/normalisation the code does.
  4. How the client reaches arm_server.py — is it RPC over a socket to a host/port?
     What host? Is that server reachable from THIS machine?

Report all four back to me before writing any motion code.

## Step 2: the ROS 1 / ROS 2 problem — decide the shape before you build

~/feeding-deployment is ROS 1 (roscore, rospy, catkin) and runs under conda envs
(`conda activate controller` for the arm). My detector is ROS 2 (rclpy).

Work out whether the arm client can be imported from a ROS 2 node in the environment
my detector runs in. Specifically: does the client path import rospy, or is it plain
Python talking to an RPC server? Do NOT import rospy into the ROS 2 node.

  - If the client is ROS-free plain Python -> it can be called from my node.
  - If it needs rospy or a different conda env -> do NOT try to bridge them. Make
    scan_pose.py a standalone script I run first, then I start the detector
    separately. Two commands is fine. Say clearly that this is what you did.

Prefer the standalone script either way if integration looks at all fragile.

## Step 3: the pose

Read off the Kinova web UI, which reports 0-360 degrees:

    J1 353.787   J2 310.642   J3 181.649   J4 234.867
    J5   2.240   J6 302.922   J7  91.168

Signed form, [-180, 180] degrees:

    [-6.213, -49.358, -178.351, -125.133, 2.240, -57.078, 91.168]

Radians:

    [-0.10844, -0.86146, -3.11281, -2.18398, 0.03910, -0.99620, 1.59118]

Already checked: J2/J4/J6 are the limited joints (+/-128.9, +/-147.8, +/-120.3 deg)
and all three are in range. J1/J3/J5/J7 are continuous. Use whichever form matches
what you found in step 1 — but prefer the signed/radian form over the raw 0-360
numbers, so a continuous joint doesn't take the long way around.

## Step 4: write it

- New file `scan_pose.py` in MY folder, with the seven angles as one named constant
  at the top. Follow the structure of retract.py as closely as you can.
- `python3 scan_pose.py` moves the arm to the pose and exits. Print the current joint
  angles, the target, and require [ENTER] to confirm before moving. Add `--yes` to
  skip the prompt and `--dry-run` to print the command without sending it.
- Move slowly — about a 5 second trajectory, not a snap move.
- After the move, read back the joint angles and verify it arrived within a small
  tolerance. Say so if it didn't.

If and only if step 2 said integration is clean: add a ROS parameter
`goto_scan_pose` to table_detector_node.py, **defaulting to false**, that moves once
at startup before the detection timer starts. Do NOT put motion inside `_tick` — that
timer fires at 2 Hz and would re-command the arm twice a second.

## Safety

- Do NOT run anything that moves the arm while exploring. When you are ready to test
  real motion, STOP and ask me — I will confirm the area is clear and hold the e-stop.
- Confirm the arm that arm_server.py controls is the same physical arm my wrist
  camera is mounted on, and not a different robot in the lab. If you can't confirm
  that from the code, ask me.
- TOMORROW.md says "don't command the arm to move unless someone is in the lab.
  Nothing below moves the arm." Update that line, it's no longer true.

## Finally

- `python3 -m py_compile` on every file you touch.
- `python3 test_placement.py` must still pass (perception-only, should be untouched).
- Add a short TOMORROW.md section: how to run scan_pose.py and any new parameter.
- Tell me what you found and what you changed.
