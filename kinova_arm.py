"""TABLE_ARM_BACKEND=kinova: the arm through RAMMP's kinova-gen3-ros2 driver (the Jetson / Sheppy).

Interface: https://rammp-org.github.io/kinova-gen3-ros2/interface
Needs the interface packages on the Python path -- source the workspace first (zsh):
    source ~/ros_ws_velocity_fix/install/setup.zsh
and RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, like the driver.

How the calls arm_backend.py lists map onto the driver:
  get_state        /joint_states (7 arm joints, effort in Nm), /ee_state (URDF fk of the tool,
                   base_link), /gripper_state (0 open .. 1 closed)
  joint moves      execute_joint_trajectory, POSITION mode. place_bowl plans and checks these
                   joint paths (bowl level, above the table), so they are sent as planned; the
                   installed driver has no speed_scale, so the waypoint times keep every joint
                   under MAX_JOINT_SPEED.
  lower_impedance  execute_joint_trajectory, IMPEDANCE mode, gains left at zero = the driver's
                   defaults (as the driver's own test client, test/send_trajectory.py, sends them).
                   Aimed below the table: the table stops the compliant arm.
  stop_action      cancels the running trajectory goal (any connection can stop it)
  set_ee_pose      go_to_ee_pose (cuRobo plans collision-free, the driver executes).
  open_gripper     a short /setpoint/gripper stream session (open_stream ... close_stream)

Two interface versions (rammp-interfaces-ros2):
  installed on Sheppy (built 2026-09-22, ~/ros_ws_velocity_fix): go_to_ee_pose takes only the
      pose -- no speed_scale / orientation_hold / impedance, it runs at cuRobo's planned speed.
  dev branch = the nightly driver images (ghcr.io/rammp-org/kinova-gen3-ros2:nightly):
      go_to_ee_pose (and go_to_joint_config) add speed_scale, orientation_hold (HOLD_LEVEL /
      HOLD_FIXED) and control_mode IMPEDANCE with named gain profiles (ImpedanceGains: SOFT /
      MEDIUM / STIFF / session default / CUSTOM); execute_joint_trajectory adds speed_scale.
  KinovaArm checks which one the sourced packages are (`compliant_goto`) and only sends the
  fields that exist. Asking for an option the driver doesn't have raises, never silently drops it.
Assumes arbitration_mode:=disabled (checked on Sheppy 2026-10-08), so the token is all zeros.
"""
import atexit
import threading
import time

import numpy as np

ARM_JOINTS = [f"joint_{i}" for i in range(1, 8)]
MAX_JOINT_SPEED = np.radians(30.0)  # rad/s, the old rig's "low" preset; every move is timed to stay under it
WAYPOINT_S = 0.5                    # s per waypoint in set_joint_trajectory (as the old kinova.py did)
CONNECT_TIMEOUT_S = 5.0
NO_TOKEN = [0] * 16                 # arbitration disabled
SENDER_ID = "rammp_table"
POSITION_MODE, IMPEDANCE_MODE = 0, 1   # ExecuteJointTrajectory control_mode
LATEST_WINS = 1                        # ... preemption
RESULT_NAMES = {0: "SUCCESSFUL", -1: "INVALID_GOAL", -4: "PATH_TOLERANCE_VIOLATED",
                -5: "GOAL_TOLERANCE_VIOLATED", -6: "PREEMPTED", -7: "PLANNING_FAILED",
                -8: "NOT_AUTHORIZED", -9: "HALTED"}
PLANNED_MOVE_TIMEOUT_S = 60.0
# dev GoToEEPose constants (same values as the .action file)
HOLD_NONE, HOLD_LEVEL, HOLD_FIXED = 0, 1, 2
PROFILE_SESSION_DEFAULT, PROFILE_SOFT, PROFILE_MEDIUM, PROFILE_STIFF = 0, 1, 2, 3
# lower_until_contact: contact = the hand stalls while the move is still running
STALL_WINDOW_S = 0.4     # s, downward speed measured over this window
STALL_FRACTION = 0.25    # stalled = speed under this fraction of the peak speed so far ...
STALL_MIN_SPEED = 0.003  # m/s, ... once it has really moved (peak above this ...
STALL_MIN_TRAVEL = 0.01  # m, ... and at least this far down)
STALL_BEFORE = 0.9       # ... and before this fraction of the move: its slow-down at the end isn't a stall
LOOSE_TOL_RAD = 0.5      # rad, path/goal tolerance for the watched impedance lowering
# lower_line_until_contact with force_z: contact = the hand's z force changes a lot
FORCE_RISE = 5.0         # N, change from the rolling baseline (or 5x its noise, if more) ...
FORCE_HITS = 3           # ... in this many readings in a row (~50 per s)
FORCE_AVG = 5            # readings averaged for "now"
FORCE_BASELINE_S = 0.4   # s of readings averaged as the baseline ...
FORCE_LAG_S = 0.3        # ... ending this long ago
FORCE_START_S = 0.5      # s into the descent before deciding
# lower_line_until_contact without force_z: contact = the hand's velocity goes to ~0 (2 cm/s descent)
MOVING_SPEED = 0.01      # m/s, it counts as moving down once faster than this ...
STILL_SPEED = 0.003      # m/s, ... then slower than this ...
STILL_HOLD_S = 0.3       # s, ... for this long = contact
HOLD_S = 0.5             # s, the hold goal's single point (the current joints)
EXIT_SETTLE_S = 2.0      # s, leaving impedance: wait at most this long for the arm to be still ...
EXIT_HOLD_S = 0.5        # s, ... then a position goal on the current joints, this long
IMPEDANCE_SETTLE_S = 1.0 # s, held in impedance at least this long before the descent ...
SETTLE_WINDOW_S = 0.5    # ... and until the end effector's z velocity has been ~0 this long
Z_STILL_SPEED = 0.002    # m/s, end effector z velocity under this = still (settle, exit, after lowering)
SETTLE_MAX_S = 8.0       # s, not still by then = don't lower
# go_to_ee_pose's goal is cuRobo's tool_frame ("fingertip midpoint; the flange sits 12 cm behind
# it along the tool axis" -- its IK_FAIL message, 2026-10-08), while /ee_state is the flange.
CUROBO_TOOL_OFFSET = 0.12   # m


def _wait(future, timeout_s):
    """Block until a future completes (the executor thread fills it in), or raise."""
    t0 = time.time()
    while not future.done():
        if time.time() - t0 > timeout_s:
            raise TimeoutError("no answer from the arm driver")
        time.sleep(0.005)
    return future.result()


class _Driver:
    """One ROS node (in its own context, so it can't clash with other rclpy use in the same
    process), spun in a background thread and shared by every connect_arm() handle."""

    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        try:
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
            from sensor_msgs.msg import JointState
            from rammp_arm_interfaces.action import ExecuteJointTrajectory, GoToEEPose
            from rammp_arm_interfaces.msg import EeState, GripperSetpoint, GripperState
            from rammp_arm_interfaces.srv import CloseStream, ListControllers, OpenStream
        except ImportError as e:
            raise SystemExit(f"{e}. Source the RAMMP interfaces first, e.g.\n"
                             "  source ~/ros_ws_velocity_fix/install/setup.zsh")
        self.msg = dict(ExecuteJointTrajectory=ExecuteJointTrajectory, GoToEEPose=GoToEEPose,
                        GripperSetpoint=GripperSetpoint,
                        CloseStream=CloseStream, ListControllers=ListControllers, OpenStream=OpenStream)
        self.ctx = Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node("table_arm_backend", context=self.ctx)
        self.joint_state = self.ee_state = self.gripper_state = None
        self.goal = None                      # handle of the running trajectory (for stop_action)
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: setattr(self, "joint_state", m), qos_profile_sensor_data)
        self.node.create_subscription(EeState, "/ee_state",
                                      lambda m: setattr(self, "ee_state", m), qos_profile_sensor_data)
        self.node.create_subscription(GripperState, "/gripper_state",
                                      lambda m: setattr(self, "gripper_state", m), qos_profile_sensor_data)
        self.trajectory = ActionClient(self.node, ExecuteJointTrajectory, "execute_joint_trajectory")
        self.go_to_ee_pose = ActionClient(self.node, GoToEEPose, "go_to_ee_pose")
        # dev / nightly interfaces? (see the module docstring)
        self.compliant_goto = hasattr(GoToEEPose.Goal(), "control_mode")
        self.trajectory_speed_scale = hasattr(ExecuteJointTrajectory.Goal(), "speed_scale")
        self.list_controllers = self.node.create_client(ListControllers, "/list_controllers")
        self.open_stream = self.node.create_client(OpenStream, "/open_stream")
        self.close_stream = self.node.create_client(CloseStream, "/close_stream")
        # Setpoint publishers must exist (and be discovered) BEFORE a stream opens.
        self.gripper_pub = self.node.create_publisher(
            GripperSetpoint, "/setpoint/gripper", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.executor = SingleThreadedExecutor(context=self.ctx)
        self.executor.add_node(self.node)
        self.spinner = threading.Thread(target=self._spin, daemon=True)
        self.spinner.start()
        # Stop the spin thread before the interpreter exits; killing it mid-spin aborts the
        # process ("terminate called without an active exception").
        atexit.register(self.close)

        t0 = time.time()
        while self.joint_state is None and time.time() - t0 < CONNECT_TIMEOUT_S:
            time.sleep(0.05)
        if self.joint_state is None:
            raise OSError("no /joint_states from kinova_gen3_node -- is the driver up, and is "
                          "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp set?")
        if not self.trajectory.wait_for_server(timeout_sec=CONNECT_TIMEOUT_S):
            raise OSError("execute_joint_trajectory action server not found")

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:
            if self.ctx.ok():       # a real failure while running, not the shutdown at exit
                raise

    def close(self):
        """At exit: stop the spin thread, then the context (it takes the node with it)."""
        self.executor.shutdown(timeout_sec=1.0)
        self.spinner.join(timeout=2.0)
        if self.ctx.ok():
            self.ctx.shutdown()


class KinovaArm:
    """What arm_backend.connect_arm() returns for TABLE_ARM_BACKEND=kinova."""

    def __init__(self):
        self.d = _Driver.get()
        self.compliant_goto = self.d.compliant_goto   # go_to_ee_pose has impedance / speed / hold

    # ---- state -------------------------------------------------------------------------
    def get_state(self):
        js = self.d.joint_state
        index = {name: i for i, name in enumerate(js.name)}
        pick = lambda values: [float(values[index[n]]) for n in ARM_JOINTS]
        state = {"position": pick(js.position), "velocity": pick(js.velocity), "effort": pick(js.effort),
                 "ee_pos": None, "gripper_pos": None}
        ee = self.d.ee_state
        if ee is not None:
            p, q = ee.pose.position, ee.pose.orientation
            state["ee_pos"] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if self.d.gripper_state is not None:
            state["gripper_pos"] = float(self.d.gripper_state.position)
        return state

    def get_speed(self):
        """There are no speed presets on this driver: every move here is timed to stay under
        MAX_JOINT_SPEED (the old "low" preset), so report "low" once the driver answers."""
        if not self.d.trajectory.server_is_ready():
            raise OSError("execute_joint_trajectory action server not available")
        return "low"

    # ---- moves -------------------------------------------------------------------------
    def _send(self, client, goal, timeout_s):
        """Send an action goal, keep its handle for stop_action, block for the result.
        Returns the driver's error_code (0 = SUCCESSFUL), or None if the goal was rejected."""
        handle = _wait(client.send_goal_async(goal), CONNECT_TIMEOUT_S)
        if not handle.accepted:
            print("arm driver rejected the goal")
            return None
        self.d.goal = handle
        try:
            result = _wait(handle.get_result_async(), timeout_s).result
        finally:
            self.d.goal = None
        if result.error_code != 0:
            print(f"move ended {RESULT_NAMES.get(result.error_code, result.error_code)}: "
                  f"{result.error_string}")
        return result.error_code

    def _trajectory_goal(self, waypoints, times, control_mode=POSITION_MODE, profile=None, loose=False,
                         start=None):
        """execute_joint_trajectory goal from `start` (default: the current joints) through
        `waypoints` at `times`.
        IMPEDANCE: the nightly driver takes a named gain profile (PROFILE_*); the older one has raw
        gains, left at zero = its defaults. loose: path/goal tolerances wide open, so the driver
        doesn't abort when the compliant arm lags or is held back -- the caller watches instead."""
        from builtin_interfaces.msg import Duration
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectoryPoint
        goal = self.d.msg["ExecuteJointTrajectory"].Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        if start is None:
            start = self.get_state()["position"]
        for q, t in [(start, 0.0)] + list(zip(waypoints, times)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.time_from_start = Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
            goal.trajectory.points.append(pt)
        goal.control_mode = control_mode
        if control_mode == IMPEDANCE_MODE and profile is not None and hasattr(goal.gains, "profile"):
            goal.gains.profile = profile
        if loose:
            tol = [JointTolerance(name=n, position=LOOSE_TOL_RAD) for n in ARM_JOINTS]
            goal.path_tolerance, goal.goal_tolerance = tol, list(tol)
        goal.preemption = LATEST_WINS
        goal.sender_id = SENDER_ID
        goal.token = NO_TOKEN
        return goal

    def _run(self, waypoints, times, control_mode=POSITION_MODE):
        """Send a trajectory from the current joints through `waypoints` at `times` (s from now)
        and block until it ends. Returns the driver's error_code (see _send)."""
        goal = self._trajectory_goal(waypoints, times, control_mode)
        return self._send(self.d.trajectory, goal, times[-1] + 30.0)

    @staticmethod
    def _timed(start, waypoints, min_step_s):
        """Times for the waypoints: each step at least min_step_s, and slow enough that no joint
        goes faster than MAX_JOINT_SPEED (x1.5 for the speed-up between waypoints)."""
        times, t, prev = [], 0.0, np.asarray(start, dtype=float)
        for q in waypoints:
            q = np.asarray(q, dtype=float)
            t += max(min_step_s, 1.5 * float(np.max(np.abs(q - prev))) / MAX_JOINT_SPEED)
            times.append(t)
            prev = q
        return times

    def set_joint_position(self, q):
        start = self.get_state()["position"]
        return self._run([q], self._timed(start, [q], min_step_s=1.0)) == 0

    def set_joint_trajectory(self, traj):
        start = self.get_state()["position"]
        return self._run(traj, self._timed(start, traj, min_step_s=WAYPOINT_S)) == 0

    def lower_impedance(self, traj, step_s):
        """Follow `traj` (joint waypoints, step_s apart) in the driver's IMPEDANCE mode. Aimed below
        the table, the compliant arm is stopped by it. Returns the driver's error_code: a stopped
        arm may well end in a tolerance error, so the caller judges contact by where the hand is."""
        start = self.get_state()["position"]
        return self._run(traj, self._timed(start, traj, min_step_s=step_s), control_mode=IMPEDANCE_MODE)

    def stop_action(self):
        """Cancel the running trajectory (works from a second connection: the goal is shared)."""
        handle = self.d.goal
        if handle is not None:
            handle.cancel_goal_async()

    def _ee_goal(self, pos, quat, hold=None, speed_scale=None, impedance=False, profile=None):
        """pos/quat: the FLANGE (end_effector_link -- what /ee_state reports and place_bowl plans).
        cuRobo's goal is its own tool_frame, CUROBO_TOOL_OFFSET further out along the tool axis
        (its own error message, 2026-10-08), so the target is moved out by that much."""
        from scipy.spatial.transform import Rotation
        axis = Rotation.from_quat([float(v) for v in quat]).as_matrix()[:, 2]
        pos = np.asarray(pos, dtype=float) + CUROBO_TOOL_OFFSET * axis
        goal = self.d.msg["GoToEEPose"].Goal()
        goal.target.header.frame_id = "base_link"
        p, q = goal.target.pose.position, goal.target.pose.orientation
        p.x, p.y, p.z = (float(v) for v in pos)
        q.x, q.y, q.z, q.w = (float(v) for v in quat)
        goal.sender_id = SENDER_ID
        goal.token = NO_TOKEN
        options = hold is not None or speed_scale is not None or impedance
        if options and not self.compliant_goto:
            raise RuntimeError("this go_to_ee_pose has no speed_scale / orientation_hold / impedance "
                               "(the installed interfaces are older than the nightly driver)")
        if hold is not None:
            goal.orientation_hold = hold
        if speed_scale is not None:
            goal.speed_scale = float(speed_scale)
        if impedance:
            goal.control_mode = IMPEDANCE_MODE
            goal.gains.profile = PROFILE_SESSION_DEFAULT if profile is None else profile
        return goal

    def set_ee_pose(self, pos, quat, hold=None, speed_scale=None, impedance=False, profile=None):
        """go_to_ee_pose: cuRobo plans, the driver executes (tool_frame to pos/quat in base_link).
        hold (HOLD_*), speed_scale, impedance + profile (PROFILE_*) need the dev / nightly
        interfaces. Returns the driver's error_code (0 = SUCCESSFUL), None if rejected."""
        goal = self._ee_goal(pos, quat, hold, speed_scale, impedance, profile)
        return self._send(self.d.go_to_ee_pose, goal, PLANNED_MOVE_TIMEOUT_S)

    def lower_until_contact(self, pos, quat, profile=None, speed_scale=None):
        """go_to_ee_pose down to pos/quat in IMPEDANCE mode, HOLD_FIXED, stopped on a stall (see
        _watch_descent). cuRobo plans it -- it may refuse a target below the arm base."""
        goal = self._ee_goal(pos, quat, HOLD_FIXED, speed_scale, True, profile)
        return self._watch_descent(self.d.go_to_ee_pose, goal, "go_to_ee_pose", PLANNED_MOVE_TIMEOUT_S)

    def lower_line_until_contact(self, line, step_s, z_planned, profile=None, force_z=None):
        """Follow `line` (place_bowl's straight line down: joint waypoints step_s apart, the first
        one the start, aimed below the table) with execute_joint_trajectory in IMPEDANCE mode --
        no planner involved. z_planned: the flange z (base_link) at each waypoint.
        1. Switch into impedance HOLDING the start for IMPEDANCE_SETTLE_S: the arm sags when the
           mode changes (~3 cm on SOFT, 2026-10-08), and that must be over before the descent,
           or it looks like motion / a stall.
        2. The descent, commanded from that same start (not from the sagged joints -- starting
           there made the first command jump back up).
        Contact = the hand's velocity goes to ~0: once moving down, under STILL_SPEED for
        STILL_HOLD_S while the move still runs (the compliant arm can't push through the table).
        (Position-based tests failed: impedance holds the hand a few cm off its command, and the
        offset drifts as it moves.) On contact the arm is held where it is, still compliant, by
        a new impedance goal at the current joints (not a cancel: after a cancel it sprang
        2.7 cm up)."""
        start, traj = line[0], line[1:]
        try:
            return self._lower_line(start, traj, step_s, z_planned, profile, force_z)
        finally:
            # However it ended (contact, no contact, not settling, an error): leave impedance
            # with the position setpoint ON the arm's current joints, so nothing jumps.
            self.exit_impedance()

    def _lower_line(self, start, traj, step_s, z_planned, profile, force_z=None):
        # 1. Impedance on, holding the start, until the hand is still (not a fixed time: on
        # 2026-10-08 it was still drifting UP after 1 s -- the driver's impedance mode holds the
        # hand a few cm off its command -- and the descent read that as contact).
        print("  impedance on, holding the start until the arm is still ...")
        settle = self._trajectory_goal([start], [SETTLE_MAX_S], IMPEDANCE_MODE, profile,
                                       loose=True, start=start)
        handle = _wait(self.d.trajectory.send_goal_async(settle), CONNECT_TIMEOUT_S)
        if not handle.accepted:
            print("arm driver rejected the impedance hold")
            return None
        self.d.goal = handle
        z_of = lambda: float(self.d.ee_state.pose.position.z)
        z_start, t0, still_since = float(z_planned[0]), time.time(), None
        while True:
            t, z = time.time() - t0, z_of()
            still_since = (still_since if still_since is not None else t) if self.z_still() else None
            if t >= IMPEDANCE_SETTLE_S and still_since is not None and t - still_since >= SETTLE_WINDOW_S:
                break
            if t > SETTLE_MAX_S - 0.5:
                self.d.goal = None
                print(f"  NOT still after {t:.1f} s in impedance mode: the hand is "
                      f"{(z - z_start) * 100:+.1f} cm from the start and still moving -- not lowering")
                return None
            time.sleep(0.02)
        lag_settled = z - z_start
        print(f"  settled after {t:.1f} s, hand {lag_settled * 100:+.1f} cm "
              f"{'above' if lag_settled >= 0 else 'below'} the planned start")
        # 2. The descent (its goal preempts the hold: LATEST_WINS).
        times = self._timed(start, traj, min_step_s=step_s)
        goal = self._trajectory_goal(traj, times, IMPEDANCE_MODE, profile, loose=True, start=start)

        def hold():
            q = self.get_state()["position"]
            g = self._trajectory_goal([q], [HOLD_S], IMPEDANCE_MODE, profile, loose=True)
            _wait(self.d.trajectory.send_goal_async(g), CONNECT_TIMEOUT_S)   # LATEST_WINS preempts

        return self._watch_descent(self.d.trajectory, goal, "execute_joint_trajectory", times[-1] + 30.0,
                                   still_test=force_z is None, force_z=force_z, on_contact=hold)

    def z_still(self):
        """The end effector's z velocity (/ee_state twist) is ~0."""
        return abs(float(self.d.ee_state.twist.linear.z)) < Z_STILL_SPEED

    def wait_z_still(self, timeout_s, hold_s=0.2):
        """Wait until the end effector's z velocity has been ~0 for hold_s (at most timeout_s)."""
        t0, since = time.time(), None
        while time.time() - t0 < timeout_s:
            since = (since or time.time()) if self.z_still() else None
            if since is not None and time.time() - since >= hold_s:
                return True
            time.sleep(0.02)
        return False

    def exit_impedance(self):
        """Back to POSITION control without a jump. In impedance the arm sits a few cm off its
        setpoint (+2.7 cm on 2026-10-08); switching to position control on that setpoint would
        pull the arm onto it (into the table, or up). So: wait until the end effector's z
        velocity is ~0, then a POSITION goal whose setpoint IS the current joints (it preempts
        the impedance goal)."""
        self.wait_z_still(EXIT_SETTLE_S)
        q = self.get_state()["position"]
        goal = self._trajectory_goal([q], [EXIT_HOLD_S], POSITION_MODE, start=q)
        code = self._send(self.d.trajectory, goal, EXIT_HOLD_S + 10.0)
        print("  impedance off: position setpoint = the current joints"
              + ("" if code == 0 else f" (driver answered {RESULT_NAMES.get(code, code)})"))
        return code == 0

    def _watch_descent(self, client, goal, what, timeout_s, still_test=False, on_contact=None,
                       force_z=None):
        """Run a downward impedance move and cancel it once the hand STALLS: it stops going down
        while the move is still running. That is the table -- the compliant arm can't push
        through it. Where the hand ends up is no test by itself: in SOFT impedance it stops short
        of its target even in free air (4 cm short on 2026-10-08). Returns True on a stall
        (contact), False if the move finished without one (nothing under the hand), None if it
        didn't run (rejected, or the driver ended it with an error -- printed)."""
        progress = {"fraction": 0.0}
        feedback = lambda f: progress.__setitem__("fraction", float(f.feedback.fraction_complete))
        handle = _wait(client.send_goal_async(goal, feedback_callback=feedback), CONNECT_TIMEOUT_S)
        if not handle.accepted:
            print(f"arm driver rejected the {what} goal")
            return None
        self.d.goal = handle
        done = handle.get_result_async()
        z_of = lambda: float(self.d.ee_state.pose.position.z)
        z0, t0 = z_of(), time.time()
        history, peak, contact = [], 0.0, False
        moved, still_since = False, None           # velocity test
        forces, hits, noise = [], 0, None          # force test
        try:
            while not done.done():
                if time.time() - t0 > timeout_s:
                    if on_contact is not None:
                        on_contact()            # hold where it is rather than cancel (it springs)
                    else:
                        handle.cancel_goal_async()
                    raise TimeoutError("impedance lowering took too long -- stopped")
                t, z = time.time(), z_of()
                history.append((t, z))
                if force_z is not None:
                    # Force test: the hand's z force, estimated from the joint torques (force_z),
                    # against a ROLLING baseline (FORCE_BASELINE_S of readings ending
                    # FORCE_LAG_S ago -- the gravity load drifts as the arm goes down). A change
                    # over FORCE_RISE for FORCE_HITS readings in a row = contact.
                    st = self.get_state()
                    forces.append((t, force_z(st["position"], st["effort"])))
                    base = [f for tt, f in forces if t - FORCE_LAG_S - FORCE_BASELINE_S <= tt <= t - FORCE_LAG_S]
                    if t - t0 > FORCE_START_S and len(base) >= 5:
                        if noise is None:
                            noise = float(np.std(base))
                            print(f"  z force {np.mean(base):+.1f} N (with gravity), noise {noise:.2f} N; "
                                  f"stop on a change over {max(FORCE_RISE, 5 * noise):.1f} N")
                        change = np.mean([f for _, f in forces[-FORCE_AVG:]]) - np.mean(base)
                        hits = hits + 1 if abs(change) > max(FORCE_RISE, 5 * noise) else 0
                        if hits >= FORCE_HITS and progress["fraction"] < 0.99:
                            print(f"  contact at z={z:.3f} (base_link): z force changed {change:+.1f} N, move "
                                  f"{progress['fraction'] * 100:.0f}% done -> holding here")
                            on_contact()
                            contact = True
                            break
                    time.sleep(0.02)
                    continue
                if still_test:
                    # Velocity test (lower_line_until_contact): the hand's own downward speed
                    # (/ee_state twist). Once it is really moving, staying under STILL_SPEED for
                    # STILL_HOLD_S while the move still runs = the table has stopped it.
                    v_down = -float(self.d.ee_state.twist.linear.z)
                    moved = moved or v_down > MOVING_SPEED
                    if moved and v_down < STILL_SPEED and progress["fraction"] < 0.99:
                        still_since = t if still_since is None else still_since
                        if t - still_since >= STILL_HOLD_S:
                            print(f"  contact at z={z:.3f} (base_link): hand still ({v_down * 100:.2f} cm/s) "
                                  f"for {STILL_HOLD_S:.1f} s, move {progress['fraction'] * 100:.0f}% done "
                                  "-> holding here")
                            on_contact()
                            contact = True
                            break
                    else:
                        still_since = None
                    time.sleep(0.02)
                    continue
                earlier = [zz for tt, zz in history if tt <= t - STALL_WINDOW_S]
                if earlier:
                    speed = (earlier[-1] - z) / STALL_WINDOW_S      # m/s downward, last window
                    peak = max(peak, speed)
                    moving = z0 - z > STALL_MIN_TRAVEL and peak > STALL_MIN_SPEED
                    if moving and speed < STALL_FRACTION * peak and progress["fraction"] < STALL_BEFORE:
                        print(f"  stall at z={z:.3f} (base_link): {speed * 100:.1f} cm/s vs peak "
                              f"{peak * 100:.1f}, move {progress['fraction'] * 100:.0f}% done -> contact, stopping")
                        handle.cancel_goal_async()
                        contact = True
                        break
                time.sleep(0.02)
            if contact and on_contact is not None:
                return True                        # the hold goal replaced the move
            result = _wait(done, 10.0).result
        finally:
            self.d.goal = None
        if contact:
            return True
        name = RESULT_NAMES.get(result.error_code, result.error_code)
        print(f"  {what} ended {name} after {progress['fraction'] * 100:.0f}% of the move: "
              f"{result.error_string or '(no message)'}; went {(z0 - z_of()) * 100:.1f} cm down, "
              f"peak {peak * 100:.1f} cm/s, hand at z={z_of():.3f} (base_link)")
        return False if result.error_code == 0 else None

    # ---- gripper -----------------------------------------------------------------------
    def open_gripper(self, timeout_s=3.0):
        """Open the gripper (position 0), checked on /gripper_state; raises if it didn't open.
        Older drivers only take gripper setpoints inside a stream session on a gripper
        controller; the nightly one lists no gripper controller (2026-10-08), so then the
        setpoint is published on its own. speed/force aren't sticky: every message has all three."""
        d = self.d
        controllers = _wait(d.list_controllers.call_async(d.msg["ListControllers"].Request()), 5.0).controllers
        names = [c.name for c in controllers
                 if c.available and any("gripper" in ch for ch in c.channels)]
        session = False
        if names:
            req = d.msg["OpenStream"].Request(controller=names[0], timeout_s=1.0, token=NO_TOKEN)
            res = _wait(d.open_stream.call_async(req), 5.0)
            if not res.accepted:
                raise RuntimeError(f"open_stream({names[0]}) refused: {res.error_code} {res.message}")
            session = True
        try:
            msg = d.msg["GripperSetpoint"](position=0.0, speed=0.5, force=0.5, token=NO_TOKEN)
            t0 = time.time()
            while time.time() - t0 < timeout_s:
                d.gripper_pub.publish(msg)
                g = d.gripper_state
                if g is not None and g.position < 0.05 and time.time() - t0 > 0.5:
                    return
                time.sleep(0.05)
        finally:
            if session:
                _wait(d.close_stream.call_async(d.msg["CloseStream"].Request(token=NO_TOKEN)), 5.0)
        g = d.gripper_state
        raise RuntimeError(f"the gripper did not open within {timeout_s:.0f} s (position "
                           f"{'?' if g is None else f'{g.position:.2f}'}, 0 = open)")

    # ---- impedance (place_bowl --impedance) is refused on this arm ----------------------
    def switch_to_joint_compliant_mode(self, *args):
        raise NotImplementedError("--impedance isn't supported on the kinova backend")

    compliant_set_joint_position = switch_out_of_compliant_mode = switch_to_joint_compliant_mode
