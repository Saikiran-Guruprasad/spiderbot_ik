#!/usr/bin/env python3
"""
placo_gait_node.py
==================
ROS 2 node that drives a 6-legged robot using the placo kinematics solver.

Subscribed topic : 'sender_data'  (jetson_stm32_poc_msgs/DataVector)
Published topic  : 'sm_shi'       (sensor_msgs/JointState)
Visualization    : MeshCat via placo_utils.visualization (robot_viz)

Gaits implemented
-----------------
1. Straight tripod   – cycloidal foot paths along the heading direction.
2. Turned tripod     – heading rotated by angle B = -atan2(x, y), same
                       cycloidal paths but the whole step frame is rotated.
3. Zero-radius turn  – alternating tripod groups rotate about the robot
                       centre by ±B degrees per half-cycle, solved via
                       placo IK with foot targets computed on an arc.

Buffer / flag / delay system
-----------------------------
Mirrors the original ROS 2 node exactly:
  • self.buffer   – latest received DataVector (updated asynchronously).
  • self.num      – command currently being executed.
  • self.delayer  – tri-state flag  0 = idle, 1 = counting, -1 = just done.
  • self.delay_s  – how many seconds to hold position after a sudden change.
  • self.is_cycle_complete – True once a full gait cycle has wrapped.
  • self.rotation_trigger / rotation_complete – rotation-gait bookkeeping.

Geometry helpers
----------------
• foot_neutral[leg]  – 3-D neutral foot position read from URDF at startup.
• rotate_around_z(pt, angle_rad, centre)  – rotates a foot target in the
  XY plane around an arbitrary centre (robot origin = [0,0,0]).
• cycloidal_swing / cycloidal_stance  – same cycloidal equations as the
  original placo demo but parameterised by heading angle B.
"""

import math
import numpy as np
import threading

import pinocchio
import placo
from placo_utils.visualization import robot_viz

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jetson_stm32_poc_msgs.msg import DataVector

# ---------------------------------------------------------------------------
# URDF / solver constants
# ---------------------------------------------------------------------------
URDF_PATH   = "/home/saikirang/placo/placo-examples/visual_urdf/urdf/visual_urdf.urdf"
LEG_NAMES   = ["Leg1", "Leg2", "Leg3", "Leg4", "Leg5", "Leg6"]
TIP_FRAMES  = {leg: f"{leg}_foot_tip" for leg in LEG_NAMES}

# Tripod groups  (indices into LEG_NAMES)
GROUP_A_IDX = [0, 2, 4]   # Leg1, Leg3, Leg5
GROUP_B_IDX = [1, 3, 5]   # Leg2, Leg4, Leg6
GROUP_A     = [LEG_NAMES[i] for i in GROUP_A_IDX]
GROUP_B     = [LEG_NAMES[i] for i in GROUP_B_IDX]

# Gait parameters
DT            = 0.01        # solver / timer period  (100 Hz)
CYCLE_TIME    = 4.0         # seconds for one full tripod cycle
STEP_SIZE     = 0.06        # forward stride length  (m)
STEP_HEIGHT   = 0.04        # foot lift height       (m)
DELAY_SECONDS = 0.22        # settle time after a sudden pose change  (≈11 × 0.02 s)

# Rotation gait
ROT_B_DEG     = 15.0        # degrees per half-rotation step
ROT_LIFT      = 0.03        # foot lift during rotation swing  (m)
READY_PHASE   = 0.5        # phase at which the ready pose is set after rotation


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def rotate_xy(point: np.ndarray, angle_rad: float) -> np.ndarray:
    """Rotate *point* (3-D) about the world Z-axis by *angle_rad*.
    The Z component is preserved."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    x =  c * point[0] - s * point[1]
    y =  s * point[0] + c * point[1]
    return np.array([x, y, point[2]])


def cycloidal_swing(neutral: np.ndarray,
                    phase: float,
                    step_size: float,
                    step_height: float,
                    heading_rad: float) -> np.ndarray:
    """
    Compute the foot position during the *swing* phase using a cycloidal path.

    phase      : 0 → 1  (fraction of the swing half-cycle)
    heading_rad: direction of travel in the world XY plane.
    """
    theta = 2.0 * math.pi * phase
    # Cycloidal displacement along heading direction
    ds = step_size * (theta - math.sin(theta)) / (2.0 * math.pi) - step_size / 2.0
    dz = step_height * (1.0 - math.cos(theta)) / 2.0

    dx = ds * math.cos(heading_rad)
    dy = ds * math.sin(heading_rad)
    return neutral + np.array([dx, dy, dz])


def cycloidal_stance(neutral: np.ndarray,
                     phase: float,
                     step_size: float,
                     heading_rad: float) -> np.ndarray:
    """
    Compute the foot position during the *stance* phase (foot on ground,
    body moves forward → foot moves backward relative to neutral).
    """
    ds = step_size / 2.0 - step_size * phase
    dx = ds * math.cos(heading_rad)
    dy = ds * math.sin(heading_rad)
    return neutral + np.array([dx, dy, 0.0])


def rotation_foot_target(neutral: np.ndarray,
                          angle_rad: float,
                          lift: float = 0.0) -> np.ndarray:
    """
    Target position for a foot in the rotation gait.
    The foot is rotated by *angle_rad* around the robot Z-axis from its
    neutral position, then lifted by *lift* (m) in Z.
    """
    rotated = rotate_xy(neutral, angle_rad)
    return rotated + np.array([0.0, 0.0, lift])


# ---------------------------------------------------------------------------
# Main ROS 2 node
# ---------------------------------------------------------------------------

class PlacoGaitNode(Node):
    """
    Hexapod gait node using placo for inverse kinematics.

    State machine
    -------------
    IDLE       – no motion command active; all feet at neutral.
    WALKING    – straight / turned tripod gait running.
    ROTATING   – zero-radius turn gait running.
    DELAY      – holding current pose while servos settle.
    """

    # ------------------------------------------------------------------ init
    def __init__(self):
        super().__init__('placo_gait_node')

        # ---- placo robot & solver ----------------------------------------
        self.robot  = placo.RobotWrapper(URDF_PATH, placo.Flags.ignore_collisions)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.solver.dt = DT
        self.robot.update_kinematics()

        # Read neutral foot positions from URDF
        pin_data = self.robot.model.createData()
        pinocchio.framesForwardKinematics(
            self.robot.model, pin_data, pinocchio.neutral(self.robot.model))
        self.foot_neutral: dict[str, np.ndarray] = {}
        for leg in LEG_NAMES:
            fid = self.robot.model.getFrameId(TIP_FRAMES[leg])
            pos = np.array(pin_data.oMf[fid].translation).flatten().copy()
            self.foot_neutral[leg] = pos
            self.get_logger().info(f"[neutral] {TIP_FRAMES[leg]}: {np.round(pos, 4)}")

        # Add one position task per foot
        self.tasks: dict[str, object] = {}
        for leg in LEG_NAMES:
            task = self.solver.add_position_task(
                TIP_FRAMES[leg], self.foot_neutral[leg].copy())
            task.configure(f"{leg}_task", "soft", 5.0)
            self.tasks[leg] = task

        # ---- MeshCat visualiser ------------------------------------------
        self.viz = robot_viz(self.robot)

        # ---- ROS publishers / subscribers --------------------------------
        self.pub = self.create_publisher(JointState, 'sm_shi', 10)

        self.buffer: DataVector | None = None   # latest received command
        self._buf_lock = threading.Lock()

        # Current active command (mirrors self.num in original)
        self.num = DataVector()
        self.num.position_x = 0.0
        self.num.position_y = 0.0
        self.num.twist_z    = 0.0

        # ---- Gait state --------------------------------------------------
        self.phase: float          = 0.25        # 0 → 1 within one CYCLE_TIME
        self.logic_phase: float     = 0.0        # phase used for logic decisions, may differ on heading change
        self.is_cycle_complete: bool = True

        # ---- Rotation state ----------------------------------------------
        self.rotation_trigger:  bool  = False
        self.rotation_complete: bool  = True
        self.rot_phase:         int   = 0       # 0-6, mirrors rotator.phase
        self.rot_B_rad:         float = 0.0     # signed rotation step (rad)
        self.rot_cycles_target: int   = 0
        self.rot_cycles_done:   int   = 0

        # ---- Delay state -------------------------------------------------
        # delayer: 0 = idle, 1 = counting down, -1 = just finished
        self.delayer:       int   = 0
        self.delay_elapsed: float = 0.0         # seconds counted so far
        self.delay_target:  float = DELAY_SECONDS

        # ---- Subscription & timer ----------------------------------------
        self.create_subscription(
            DataVector, 'sender_data', self._cb_buffer, 10)

        self.timer = self.create_timer(DT, self._tick)
        self.get_logger().info('PlacoGaitNode ready — waiting for commands.')

    # --------------------------------------------------------------- helpers

    def _heading(self) -> float:
        """Direction of travel in radians from position_x / position_y."""
        return math.atan2(self.num.position_x, self.num.position_y)   # note: atan2(x,y) mirrors original

    def _moving(self) -> bool:
        return (self.num.position_x != 0.0 or
                self.num.position_y != 0.0 or
                self.num.twist_z    != 0.0)

    def _command_changed(self) -> bool:
        with self._buf_lock:
            if self.buffer is None:
                return False
            return (self.buffer.position_x != self.num.position_x or
                    self.buffer.position_y != self.num.position_y or
                    self.buffer.twist_z    != self.num.twist_z)

    def _set_all_feet_neutral(self):
        for leg in LEG_NAMES:
            self.tasks[leg].target_world = self.foot_neutral[leg].copy()

    def _solve_and_publish(self):
        """Run one placo solver step, visualise, then publish JointState."""
        self.solver.solve(True)
        self.robot.update_kinematics()
        self.viz.display(self.robot.state.q)
        self._publish_joint_state()

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = [str(self.robot.model.names[i])
                        for i in range(1, self.robot.model.njoints)]
        msg.position = self.robot.state.q[7:].tolist()   # skip freeflyer
        self.pub.publish(msg)

    # ---------------------------------------------------------- delay system

    def _start_delay(self, seconds: float = DELAY_SECONDS):
        """Begin a timed hold.  delayer = 1 while counting."""
        self.delayer       = 1
        self.delay_target  = seconds
        self.delay_elapsed = 0.0

    def _tick_delay(self) -> bool:
        """
        Advance the delay counter by one DT.
        Returns True while still counting, False when the delay has finished
        (at which point self.delayer is set to -1).
        """
        self.delay_elapsed += DT
        if self.delay_elapsed >= self.delay_target:
            self.delayer = -1
            return False          # delay just expired
        return True               # still waiting

    # ------------------------------------------------------ buffer callback

    def _cb_buffer(self, msg: DataVector):
        with self._buf_lock:
            self.buffer = msg

    # ---------------------------------------------------------- main tick

    def _tick(self):
        # ── 1. Handle active delay ────────────────────────────────────────
        if self.delayer == 1:
            if self._tick_delay():
                return                  # still counting — do nothing else
            # delay just finished (delayer is now -1); fall through

        # ── 2. Check for a new command ────────────────────────────────────
        if self._command_changed():
            with self._buf_lock:
                buf = self.buffer

            can_switch = (self.is_cycle_complete or
                          not self._moving() or
                          self.rotation_complete)

            if can_switch:
                # Always snap to neutral when switching — this cleanly handles
                # the stop case (0,0,0) and any direction change. The only
                # exception is a same-heading speed change, which doesn't exist
                # in this protocol anyway. Snapping guarantees feet are at a
                # known position before any new gait starts.
                stopping = (buf.position_x == 0.0 and
                            buf.position_y == 0.0 and
                            buf.twist_z    == 0.0)

                # Skip snap only when continuing the same axis (original logic)
                same_x = (buf.position_x == self.num.position_x)
                same_y = (buf.position_y == self.num.position_y)
                skip_snap = (
                    not stopping and (
                        (same_y and buf.position_x == 0.0) or
                        (same_x and buf.position_y == 0.0 and
                         (buf.position_x != 0.0 or buf.position_y != 0.0))
                    )
                )

                if not skip_snap:
                    self._set_all_feet_neutral()
                    self._solve_and_publish()
                    if self.delayer == 0:
                        self._start_delay()
                        return
                    else:   # delayer == -1: reset and continue
                        self.delayer = 0

                # Commit the new command
                self.num.position_x = buf.position_x
                self.num.position_y = buf.position_y
                self.num.twist_z    = buf.twist_z
                self.is_cycle_complete = False

                # If stopping, mark complete so next command switches instantly
                if stopping:
                    self.is_cycle_complete = True
                    self.rotation_trigger  = False
                    self.rotation_complete = True
                    return

                if buf.twist_z != 0.0:
                    # ── Switch into rotation gait ──────────────────────
                    self.rotation_trigger  = True
                    self.rotation_complete = False
                    self.rot_phase         = 0
                    self.rot_cycles_done   = 0

                    # Compute rotation parameters (mirror of Rotation class)
                    final_angle_deg  = -math.degrees(math.atan2(buf.position_x,
                                                                 buf.position_y))
                    self.rot_B_rad   = math.radians(
                        ROT_B_DEG if final_angle_deg > 0 else -ROT_B_DEG)
                    self.rot_cycles_target = max(
                        1, round(abs(final_angle_deg / (2.0 * ROT_B_DEG))))

                    # Raise alternating tripod to travel height
                    for leg in GROUP_A:
                        self.tasks[leg].target_world = (
                            self.foot_neutral[leg] + np.array([0, 0, ROT_LIFT]))
                    for leg in GROUP_B:
                        self.tasks[leg].target_world = self.foot_neutral[leg].copy()
                    self._solve_and_publish()

                    if self.delayer == 0:
                        self._start_delay()
                        return
                    else:
                        self.delayer = 0
                    return

                else:
                    # ── Switch into walking gait ───────────────────────
                    self.rotation_trigger  = False
                    self.rotation_complete = False
                    # Start new gait at current phase so there is no
                    # discontinuity — feet continue from where they are.
                    self._init_walking(start_phase=self.phase)

        # ── 3. Rotation gait ──────────────────────────────────────────────
        if self.rotation_trigger:
            self._tick_rotation()
            return

        # ── 4. Walking gait ───────────────────────────────────────────────
        # NOTE: do NOT reset is_cycle_complete here — it is set in
        # _tick_walking() on phase wrap and must survive until _tick() reads it.
        if self._moving() and not self.rotation_trigger:
            self._tick_walking()

    # --------------------------------------------------- walking gait logic

    def _init_walking(self, start_phase: float = 0.25):
        """
        Set initial foot targets consistent with *start_phase* so there is no
        positional discontinuity when switching gaits mid-cycle.

        start_phase = 0.0        -> fresh start.
        start_phase = self.phase -> continue from current cycle position,
                                    used on heading/command change mid-walk.
        """
        heading = self._heading()
        self.phase = start_phase

        if self.phase < 0.5:
            p = self.phase / 0.5
            for leg in GROUP_A:
                self.tasks[leg].target_world = cycloidal_swing(
                    self.foot_neutral[leg], p, STEP_SIZE, STEP_HEIGHT, heading)
            for leg in GROUP_B:
                self.tasks[leg].target_world = cycloidal_stance(
                    self.foot_neutral[leg], p, STEP_SIZE, heading)
        else:
            p = (self.phase - 0.5) / 0.5
            for leg in GROUP_B:
                self.tasks[leg].target_world = cycloidal_swing(
                    self.foot_neutral[leg], p, STEP_SIZE, STEP_HEIGHT, heading)
            for leg in GROUP_A:
                self.tasks[leg].target_world = cycloidal_stance(
                    self.foot_neutral[leg], p, STEP_SIZE, heading)

    def _tick_walking(self):
        """
        Advance gait phase by DT / CYCLE_TIME and update foot targets.
        Phase 0.0 → 0.5 : Group A swings, Group B in stance.
        Phase 0.5 → 1.0 : Group B swings, Group A in stance.
        Wraps to 0 at 1.0 (cycle complete flag set, NOT cleared here —
        _tick() reads it before calling us again).
        """
        self.is_cycle_complete = False
        heading = self._heading()
        
        prev_logic_phase = self.logic_phase
        self.phase = (self.phase + DT / CYCLE_TIME) % 1.0
        self.logic_phase = (self.phase + 0.25) % 1.0   # logic phase is shifted by 0.25 to match original

        # Detect phase wrap → one full cycle just finished
        if self.logic_phase < prev_logic_phase:
            self.is_cycle_complete = True

        if self.phase < 0.5:
            p = self.phase / 0.5          # 0 → 1 within swing
            for leg in GROUP_A:
                self.tasks[leg].target_world = cycloidal_swing(
                    self.foot_neutral[leg], p, STEP_SIZE, STEP_HEIGHT, heading)
            for leg in GROUP_B:
                self.tasks[leg].target_world = cycloidal_stance(
                    self.foot_neutral[leg], p, STEP_SIZE, heading)
        else:
            p = (self.phase - 0.5) / 0.5
            for leg in GROUP_B:
                self.tasks[leg].target_world = cycloidal_swing(
                    self.foot_neutral[leg], p, STEP_SIZE, STEP_HEIGHT, heading)
            for leg in GROUP_A:
                self.tasks[leg].target_world = cycloidal_stance(
                    self.foot_neutral[leg], p, STEP_SIZE, heading)

        self._solve_and_publish()

    # -------------------------------------------------- rotation gait logic

    # ── rotation gait ─────────────────────────────────────────────────────
 
    def _tick_rotation(self):
        """
        Faithful reproduction of the original rotation_trigger phase sequence.
 
        The original uses GROUND SLIDES — one tripod drags along the ground
        to rotate the body while the other tripod either resets (lifts then
        returns to neutral) or holds. There is NO lift during the rotation
        move itself. The lift is only used to reset a foot back to neutral
        so it can slide again on the next cycle.
 
        Initial pose (set before entering this loop):
          GROUP_A → neutral + [0,0,ROT_LIFT]   (lifted)
          GROUP_B → neutral on ground
 
        Phase 0: GROUP_B slides on ground to angle +B  (body rotates)
                 GROUP_A comes down to neutral ground
        Phase 1: GROUP_A stays neutral (explicit, mirrors original)
                 GROUP_B stays at +B ground (implicit)
        Phase 2: GROUP_B lifts (reset move — foot off ground)
                 GROUP_A stays neutral ground
        Phase 3: GROUP_A slides on ground to angle +B  (body rotates again)
                 GROUP_B stays lifted
        Phase 4: GROUP_B comes back down to neutral ground
                 GROUP_A stays at +B
        Phase 5: GROUP_A lifts (reset move)
                 GROUP_B stays neutral
        Phase 6: Cycle count / exit decision
        """
        B = self.rot_B_rad
 
        if self.rot_phase == 0:
            # GROUP_B slides to +B on ground; GROUP_A lands at neutral
            for leg in GROUP_B:
                self.tasks[leg].target_world = rotation_foot_target(
                    self.foot_neutral[leg], B, lift=0.0)
            for leg in GROUP_A:
                self.tasks[leg].target_world = self.foot_neutral[leg].copy()
 
        elif self.rot_phase == 1:
            # GROUP_A explicit neutral (mirrors original); GROUP_B stays at +B
            for leg in GROUP_A:
                self.tasks[leg].target_world = self.foot_neutral[leg].copy()
 
        elif self.rot_phase == 2:
            # GROUP_B lifts to reset; GROUP_A stays neutral
            for leg in GROUP_B:
                self.tasks[leg].target_world = (
                    self.foot_neutral[leg] + np.array([0.0, 0.0, ROT_LIFT]))
 
        elif self.rot_phase == 3:
            # GROUP_A slides to +B on ground; GROUP_B stays lifted
            for leg in GROUP_A:
                self.tasks[leg].target_world = rotation_foot_target(
                    self.foot_neutral[leg], B, lift=0.0)
 
        elif self.rot_phase == 4:
            # GROUP_B comes back to neutral ground; GROUP_A stays at +B
            for leg in GROUP_B:
                self.tasks[leg].target_world = self.foot_neutral[leg].copy()
 
        elif self.rot_phase == 5:
            # GROUP_A lifts to reset; GROUP_B stays neutral
            for leg in GROUP_A:
                self.tasks[leg].target_world = (
                    self.foot_neutral[leg] + np.array([0.0, 0.0, ROT_LIFT]))
 
        elif self.rot_phase == 6:
            self.rot_cycles_done += 1
 
            with self._buf_lock:
                buf = self.buffer
 
            buf_unchanged = (buf is not None and
                             buf.position_x == self.num.position_x and
                             buf.position_y == self.num.position_y and
                             buf.twist_z    == self.num.twist_z)
 
            if self.rot_cycles_done >= self.rot_cycles_target and buf_unchanged:
                # Done rotating → ready pose aligned to forward walk → walk
                # Default to y=1 forward heading after rotation completes,
                # same as original which sets num.position_y=1 after rotation.
                post_rot_heading = math.atan2(0.0, 1.0)   # = 0 rad, facing +Y
                self._set_all_feet_neutral()
                self._solve_and_publish()
                self.phase = READY_PHASE
 
                self.rotation_trigger  = False
                self.rotation_complete = True
                self.rot_phase         = 0
 
                self.num.position_x = 0.0
                self.num.position_y = 1.0
                self.num.twist_z    = 0.0
                if buf is not None:
                    buf.position_x = 0.0
                    buf.position_y = 1.0
                    buf.twist_z    = 0.0
                    with self._buf_lock:
                        self.buffer = buf
 
                self.rotation_complete = False
                self._init_walking()
 
                if self.delayer == 0:
                    self._start_delay()
                    return
                else:
                    self.delayer = 0
                return
 
            elif (buf is not None and
                  (buf.position_x != self.num.position_x or
                   buf.position_y != self.num.position_y or
                   buf.twist_z    != self.num.twist_z) and
                  buf.twist_z == 0.0):
                # Abort rotation → ready pose → walk on new heading
                new_heading = math.atan2(buf.position_x, buf.position_y)
                self._set_all_feet_neutral()
                self._solve_and_publish()
                self.phase = READY_PHASE
 
                self.rotation_trigger  = False
                self.rotation_complete = True
                self.rot_phase         = 0
 
                self.num.position_x = buf.position_x
                self.num.position_y = buf.position_y
                self.num.twist_z    = 0.0
 
                self.rotation_complete = False
                self._init_walking()
 
                if self.delayer == 0:
                    self._start_delay()
                    return
                else:
                    self.delayer = 0
                return
 
            else:
                self.rot_phase = -1   # incremented to 0 below
 
        # Publish and advance phase
        self._solve_and_publish()
 
        if self.delayer == 0:
            self._start_delay()
            return
        elif self.delayer == -1:
            self.delayer = 0
 
        self.rot_phase += 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PlacoGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()