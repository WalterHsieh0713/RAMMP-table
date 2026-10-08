"""Which arm stack the scripts talk to, picked by TABLE_ARM_BACKEND (default "feeding").

place_bowl.py and scan_pose.py only use connect_arm() / describe() from here, so running on a
different stack (e.g. the Jetson brought up by Sheppy) means adding one backend below; the
planning and touch-sensing code stays as it is.

What a backend's connect_arm() must return -- an object with:
  get_state() -> dict with
      "position"   7 joint angles, rad
      "velocity"   7 joint speeds, rad/s
      "effort"     7 joint torques, Nm      (touch sensing compares these)
      "ee_pos"     tool pose as reported by the arm: [x, y, z, qx, qy, qz, qw] in the base frame
      "gripper_pos"  gripper opening (scan_pose.py checks it is open)
  get_speed() -> "low" / ... ; raise if the arm won't accept commands
  set_joint_position(q)       blocking joint move, rad; returns truthy on success
  set_joint_trajectory(traj)  blocking; place_bowl's continuous lowering (see joint_move_slow)
  set_ee_pose(pos, quat)      blocking Cartesian move (the back-off after release)
  stop_action()               stop the running move -- called from the second connection
  open_gripper()
  switch_to_joint_compliant_mode() / compliant_set_joint_position(q) / switch_out_of_compliant_mode()
      only for --impedance, which is refused on this arm; a backend may raise NotImplementedError.
It must also survive two connections at once: place_bowl reads torques on a second
connection while the first one blocks in set_joint_trajectory.

Backends:
  feeding  ~/feeding-deployment's arm_server.py over TCP (rchi-cpu-5; ARM_RPC_HOST, default
           127.0.0.1). Needs bulldog_bypass.py + the stub base server running there.
  kinova   RAMMP's kinova-gen3-ros2 driver over ROS 2 (the Jetson / Sheppy); see kinova_arm.py.
           Source the interfaces first: source ~/ros_ws_velocity_fix/install/setup.zsh
"""
import os

BACKEND = os.environ.get("TABLE_ARM_BACKEND", "feeding")

# This rig (rchi-cpu-5) runs arm_server locally -- no NUC. arm_interface reads
# ARM_RPC_HOST at import time, so default it here; export it to override.
os.environ.setdefault("ARM_RPC_HOST", "127.0.0.1")


def describe():
    """One line for the printout: which stack, at which address."""
    if BACKEND == "feeding":
        from feeding_deployment.control.robot_controller.arm_interface import ARM_RPC_PORT, NUC_HOSTNAME
        return f"arm_server at {NUC_HOSTNAME}:{ARM_RPC_PORT} (set ARM_RPC_HOST to change)"
    if BACKEND == "kinova":
        return "kinova_gen3_node over ROS 2 (execute_joint_trajectory, our own timing)"
    return f"backend '{BACKEND}'"


def connect_arm():
    """The arm proxy, or raise OSError if the stack isn't running."""
    if BACKEND == "feeding":
        from feeding_deployment.control.robot_controller.arm_interface import (
            ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager)
        ArmManager.register("ArmInterface")
        manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        manager.connect()
        return manager.ArmInterface()
    if BACKEND == "kinova":
        from kinova_arm import KinovaArm
        return KinovaArm()
    raise SystemExit(f"TABLE_ARM_BACKEND={BACKEND!r} is not written yet (known: feeding, kinova).")
