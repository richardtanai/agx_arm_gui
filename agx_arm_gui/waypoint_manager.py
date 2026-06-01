"""Waypoint recording & playback for the AGX arm.

Runs an rclpy node in its own background thread so the GUI does not block.
- Subscribes to feedback/joint_states (which already includes the gripper joint
  when publish_gripper_joint=true) and feedback/gripper_status (gripper width).
- Records a waypoint by snapshotting the latest joint positions.
- Plays back a sequence by republishing on control/joint_states at a fixed rate,
  scaled by a user-controlled speed factor.

YAML schema (versioned so older files can be detected later):

    version: 1
    arm_type: piper
    joint_names: [joint1, joint2, ..., gripper]
    waypoints:
      - name: wp_1
        positions: [0.0, 0.1, ...]
        gripper_width: 0.04        # metres; null if not available
        hold_time: 1.0             # seconds to dwell on this waypoint
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Callable, Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Empty, String
from std_srvs.srv import SetBool
from std_srvs.srv import Empty as EmptySrv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from .config_loader import load_config


@dataclass
class Waypoint:
    name: str
    positions: list                       # joint positions in joint_names order
    gripper_width: Optional[float] = None # metres; from feedback/gripper_status
    hold_time: float = 1.0                # seconds before transitioning to next


@dataclass
class WaypointSequence:
    arm_type: str = "piper"
    joint_names: list = field(default_factory=list)
    waypoints: list = field(default_factory=list)  # list[Waypoint]
    version: int = 1

    def to_dict(self) -> dict:
        return {
            "version": self.version,
            "arm_type": self.arm_type,
            "joint_names": list(self.joint_names),
            "waypoints": [asdict(wp) for wp in self.waypoints],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "WaypointSequence":
        wps = [Waypoint(**wp) for wp in d.get("waypoints", [])]
        return cls(
            arm_type=d.get("arm_type", "piper"),
            joint_names=list(d.get("joint_names", [])),
            waypoints=wps,
            version=int(d.get("version", 1)),
        )


class _RosWorker(Node):
    """The rclpy half — subscriptions, publisher, playback timer."""

    PLAYBACK_RATE_HZ = 50.0  # publish rate during playback

    def __init__(self):
        super().__init__("agx_waypoint_manager")

        self._lock = threading.Lock()
        self._latest_names: list = []
        self._latest_positions: list = []
        self._latest_gripper_width: Optional[float] = None

        self.create_subscription(
            JointState, "feedback/joint_states", self._joint_cb, 10
        )

        # GripperStatus is in agx_arm_msgs — import lazily so the GUI still
        # imports cleanly on systems where the message package is unavailable
        # (e.g. running the GUI without sourcing the ROS overlay).
        try:
            from agx_arm_msgs.msg import GripperStatus
            self.create_subscription(
                GripperStatus, "feedback/gripper_status",
                self._gripper_cb, 10,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"GripperStatus unavailable, gripper width will not be recorded: {exc}"
            )

        self._cmd_pub = self.create_publisher(JointState, "control/joint_states", 10)

        # Service clients — created lazily; controller usually starts after the GUI.
        self._enable_cli      = self.create_client(SetBool,  "enable_agx_arm")
        self._move_home_cli   = self.create_client(EmptySrv, "move_home")

        # Action client for MoveIt's joint trajectory controller. When it's
        # available we use it for playback (avoids the joint_state_broadcaster
        # collision on control/joint_states). Otherwise we fall through to
        # direct JointState publishing.
        self._traj_action = ActionClient(self, FollowJointTrajectory,
                                         "/arm_controller/follow_joint_trajectory")
        self._gripper_action = ActionClient(self, FollowJointTrajectory,
                                            "/gripper_controller/follow_joint_trajectory")
        self._active_traj_goal_handle = None
        self._traj_seq_running = False
        self._traj_seq_index = 0
        self._traj_seq = None
        self._traj_seq_progress_cb = None
        self._traj_seq_done_cb = None
        self._traj_seq_speed = 1.0

        # Playback state — accessed only on the executor thread.
        self._playback_seq: Optional[WaypointSequence] = None
        self._playback_idx: int = 0
        self._playback_speed: float = 1.0
        self._playback_phase_start: float = 0.0
        self._playback_running: bool = False
        self._playback_paused: bool = False
        self._playback_done_cb: Optional[Callable[[bool, str], None]] = None
        self._playback_progress_cb: Optional[Callable[[int, int], None]] = None

        # IIoT bridge — toggled from the GUI. Resolves target_id → YAML file
        # via a callback the GUI installs (so it can swap the map at runtime).
        self._iiot_enabled: bool = False
        self._iiot_resolve_target = None         # Callable[[int], Optional[Path]]
        self._iiot_default_speed: float = 1.0
        self._iiot_active_target: Optional[int] = None
        self._iiot_event_cb = None               # GUI hook for log lines

        self._iiot_status_pub = self.create_publisher(String, "iiot/status", 10)
        # Target waypoint pose (JSON) — published whenever the manager starts
        # working on a new waypoint, so the SPB bridge can mirror the pose
        # SCADA-side as Positions/Target/* metrics.
        self._iiot_target_pub = self.create_publisher(String, "iiot/target_waypoint", 10)
        self.create_subscription(Int32, "iiot/execute", self._iiot_execute_cb, 10)
        self.create_subscription(Empty, "iiot/halt",    self._iiot_halt_cb,    10)

        self.create_timer(1.0 / self.PLAYBACK_RATE_HZ, self._playback_tick)
        # Periodic "ready" heartbeat so the SPB bridge knows the manager is alive.
        self.create_timer(2.0, self._iiot_heartbeat)

    # ── Subscriptions ────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState):
        with self._lock:
            self._latest_names = list(msg.name)
            self._latest_positions = list(msg.position)

    def _gripper_cb(self, msg):
        with self._lock:
            self._latest_gripper_width = float(msg.width)

    # ── Snapshot for record ──────────────────────────────────────────────

    def snapshot(self) -> Optional[dict]:
        """Return current joint state, or None if nothing has arrived yet."""
        with self._lock:
            if not self._latest_names:
                return None
            return {
                "names": list(self._latest_names),
                "positions": list(self._latest_positions),
                "gripper_width": self._latest_gripper_width,
            }

    # ── Playback control ─────────────────────────────────────────────────

    def start_playback(
        self,
        seq: WaypointSequence,
        speed: float,
        progress_cb: Callable[[int, int], None],
        done_cb: Callable[[bool, str], None],
    ):
        if not seq.waypoints:
            done_cb(False, "Sequence has no waypoints")
            return

        # Always use the FollowJointTrajectory action. The direct-publish
        # fallback was removed because it conflicts with MoveIt's
        # joint_state_broadcaster (200 Hz flood drowns out 50 Hz commands)
        # and bypasses velocity/acceleration limits enforced by the controller.
        # If the action server isn't up yet (MoveIt still starting), wait a
        # short grace period before failing.
        if not self._traj_action.server_is_ready():
            self.get_logger().info(
                "Waiting up to 2 s for /arm_controller/follow_joint_trajectory "
                "action server..."
            )
            if not self._traj_action.wait_for_server(timeout_sec=2.0):
                done_cb(False,
                        "FollowJointTrajectory action not available — "
                        "is MoveIt2 (or the controller stack) running?")
                return

        self._start_trajectory_playback(seq, speed, progress_cb, done_cb)

    def stop_playback(self):
        if self._traj_seq_running:
            # Cancel any in-flight FollowJointTrajectory goal first.
            if self._active_traj_goal_handle is not None:
                try:
                    self._active_traj_goal_handle.cancel_goal_async()
                except Exception:
                    pass
            self._finish_traj_playback(False, "Stopped by user")
        if self._playback_running:
            self._playback_running = False
            cb = self._playback_done_cb
            self._playback_done_cb = None
            self._playback_progress_cb = None
            if cb:
                cb(False, "Stopped by user")

    def set_speed(self, speed: float):
        self._playback_speed = max(0.05, float(speed))

    def set_paused(self, paused: bool):
        if not self._playback_running:
            return
        if paused and not self._playback_paused:
            self._playback_paused = True
            self._pause_t = time.monotonic()
        elif not paused and self._playback_paused:
            # Carry the elapsed-zero forward so dwell time isn't shortened.
            self._playback_phase_start += time.monotonic() - self._pause_t
            self._playback_paused = False

    def is_playing(self) -> bool:
        return self._playback_running or self._traj_seq_running

    def _playback_tick(self):
        if not self._playback_running or self._playback_paused:
            return
        seq = self._playback_seq
        if seq is None or self._playback_idx >= len(seq.waypoints):
            self._finish_playback(True, "Playback complete")
            return

        wp = seq.waypoints[self._playback_idx]

        # Always re-publish current target so the controller keeps tracking.
        self._publish_waypoint(seq.joint_names, wp)

        elapsed = time.monotonic() - self._playback_phase_start
        scaled_hold = wp.hold_time / self._playback_speed
        if elapsed >= scaled_hold:
            self._playback_idx += 1
            self._playback_phase_start = time.monotonic()
            if self._playback_progress_cb:
                self._playback_progress_cb(self._playback_idx, len(seq.waypoints))

    def _publish_waypoint(self, joint_names: list, wp: Waypoint):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joint_names)
        msg.position = [float(p) for p in wp.positions]
        self._cmd_pub.publish(msg)

    def control_subscriber_count(self) -> int:
        """How many nodes are subscribed to control/joint_states right now."""
        return self.count_subscribers("control/joint_states")

    def call_enable_arm(self, enable: bool, callback):
        """Async call to /enable_agx_arm. callback(ok: bool, msg: str)."""
        if not self._enable_cli.service_is_ready():
            # Try to refresh service availability (cheap, non-blocking).
            self._enable_cli.wait_for_service(timeout_sec=0.0)
            if not self._enable_cli.service_is_ready():
                callback(False, "service /enable_agx_arm not available — is the arm controller running?")
                return
        req = SetBool.Request()
        req.data = bool(enable)
        future = self._enable_cli.call_async(req)

        def _done(f):
            try:
                resp = f.result()
                callback(bool(resp.success), resp.message or ("enabled" if enable else "disabled"))
            except Exception as exc:
                callback(False, str(exc))

        future.add_done_callback(_done)

    def _finish_playback(self, ok: bool, reason: str):
        self._playback_running = False
        cb = self._playback_done_cb
        self._playback_done_cb = None
        self._playback_progress_cb = None
        if cb:
            cb(ok, reason)

    # ── Trajectory-action playback (used when MoveIt is up) ──────────────

    def _start_trajectory_playback(self, seq, speed, progress_cb, done_cb):
        self._traj_seq = seq
        self._traj_seq_index = 0
        self._traj_seq_speed = max(0.05, float(speed))
        self._traj_seq_progress_cb = progress_cb
        self._traj_seq_done_cb = done_cb
        self._traj_seq_running = True
        progress_cb(0, len(seq.waypoints))
        self._send_next_traj_goal()

    def _send_next_traj_goal(self):
        if not self._traj_seq_running:
            return
        if self._traj_seq_index >= len(self._traj_seq.waypoints):
            self._finish_traj_playback(True, "Playback complete")
            return

        wp = self._traj_seq.waypoints[self._traj_seq_index]
        # Filter to arm joints only — joint_trajectory_controller named
        # arm_controller does not own the gripper. The gripper is dispatched
        # separately on control/joint_states; see _send_gripper_command.
        names_in = list(self._traj_seq.joint_names)
        positions_in = list(wp.positions)
        traj_names = []
        traj_positions = []
        gripper_traj_names: list = []
        gripper_traj_positions: list = []
        gripper_position: Optional[float] = None  # legacy "gripper" joint fallback
        for n, p in zip(names_in, positions_in):
            if n.startswith("joint"):
                traj_names.append(n)
                traj_positions.append(float(p))
            elif n == "gripper":
                gripper_position = float(p)
            elif n in ("gripper_joint1", "gripper_joint2"):
                gripper_traj_names.append(n)
                gripper_traj_positions.append(float(p))
        if not traj_names:
            self._finish_traj_playback(False, "No arm joints in sequence")
            return

        secs = max(wp.hold_time / self._traj_seq_speed, 0.05)
        sec_int = int(secs)
        nsec_int = int((secs - sec_int) * 1e9)

        # Gripper in parallel with arm — prefer gripper_controller action (same
        # path MoveIt uses); fall back to control/joint_states for legacy files.
        if gripper_traj_names and self._gripper_action.server_is_ready():
            self._send_gripper_traj(gripper_traj_names, gripper_traj_positions, secs)
        else:
            if gripper_position is None and gripper_traj_names:
                gripper_position = abs(gripper_traj_positions[0]) * 2.0
            self._send_gripper_command(gripper_position, wp.gripper_width)

        traj = JointTrajectory()
        traj.joint_names = traj_names
        point = JointTrajectoryPoint()
        point.positions = traj_positions
        point.time_from_start = Duration(sec=sec_int, nanosec=nsec_int)
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._traj_action.send_goal_async(goal)
        future.add_done_callback(self._on_traj_goal_response)

    def _on_traj_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._finish_traj_playback(False, f"Goal send failed: {exc}")
            return
        if handle is None or not handle.accepted:
            self._finish_traj_playback(False, "Goal rejected by controller")
            return
        self._active_traj_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_traj_result)

    def _on_traj_result(self, future):
        if not self._traj_seq_running:
            return
        try:
            wrapped = future.result()
            status = wrapped.status   # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
            error = wrapped.result.error_code
        except Exception as exc:
            self._finish_traj_playback(False, f"Goal result failed: {exc}")
            return

        # 4 == GoalStatus.STATUS_SUCCEEDED. We accept it as success; the
        # controller's error_code is a secondary check used only for logs.
        if status != 4:
            self._finish_traj_playback(False, f"Trajectory aborted (status={status}, err={error})")
            return

        self._traj_seq_index += 1
        if self._traj_seq_progress_cb:
            self._traj_seq_progress_cb(self._traj_seq_index, len(self._traj_seq.waypoints))
        self._send_next_traj_goal()

    def _finish_traj_playback(self, ok: bool, reason: str):
        self._traj_seq_running = False
        self._active_traj_goal_handle = None
        cb = self._traj_seq_done_cb
        self._traj_seq_done_cb = None
        self._traj_seq_progress_cb = None
        if cb:
            cb(ok, reason)

    def _send_gripper_traj(self, names: list, positions: list, secs: float):
        """Send gripper joints to gripper_controller/follow_joint_trajectory (fire-and-forget)."""
        sec_int = int(secs)
        nsec_int = int((secs - sec_int) * 1e9)
        traj = JointTrajectory()
        traj.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=sec_int, nanosec=nsec_int)
        traj.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._gripper_action.send_goal_async(goal)

    def _send_gripper_command(self, joint_pos: Optional[float], width_m: Optional[float]):
        """Dispatch the gripper width separately from the arm trajectory.

        arm_controller_single_node dispatches gripper.move(width=...) for any
        message on control/joint_states that contains a 'gripper' joint, so
        we publish a single-joint JointState here. This is disjoint from the
        FollowJointTrajectory goal (which only carries joint1..joint6) and so
        avoids the broadcaster contention that motivated removing direct
        publishing for the arm.
        """
        value = joint_pos if joint_pos is not None else width_m
        if value is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper"]
        msg.position = [float(value)]
        self._cmd_pub.publish(msg)

    # ── Move-to-home service call ────────────────────────────────────────

    def call_move_home(self, callback):
        """Async call to /move_home. callback(ok: bool, msg: str)."""
        if not self._move_home_cli.service_is_ready():
            self._move_home_cli.wait_for_service(timeout_sec=0.0)
            if not self._move_home_cli.service_is_ready():
                callback(False, "service /move_home not available — is the arm controller running?")
                return
        future = self._move_home_cli.call_async(EmptySrv.Request())

        def _done(f):
            try:
                f.result()
                callback(True, "moved to home")
            except Exception as exc:
                callback(False, str(exc))
        future.add_done_callback(_done)

    # ── IIoT bridge ──────────────────────────────────────────────────────

    def configure_iiot(self, enabled: bool, resolve_target, default_speed: float, event_cb=None):
        self._iiot_enabled = bool(enabled)
        self._iiot_resolve_target = resolve_target
        self._iiot_default_speed = max(0.05, float(default_speed))
        self._iiot_event_cb = event_cb
        self._iiot_publish("ready" if enabled else "disabled")

    def _iiot_publish(self, text: str):
        msg = String()
        msg.data = text
        self._iiot_status_pub.publish(msg)
        if self._iiot_event_cb:
            try:
                self._iiot_event_cb(text)
            except Exception:
                pass

    def _iiot_heartbeat(self):
        if self._iiot_enabled and not (self._playback_running or self._traj_seq_running):
            self._iiot_publish("ready")

    def _iiot_execute_cb(self, msg: Int32):
        target = int(msg.data)
        if not self._iiot_enabled:
            self._iiot_publish(f"error:iiot_disabled (target={target})")
            return
        if self._playback_running or self._traj_seq_running:
            self._iiot_publish(f"error:busy (target={target})")
            return
        seq = self._iiot_load_for_target(target)
        if seq is None:
            self._iiot_publish(f"error:no_sequence (target={target})")
            return
        self._iiot_active_target = target
        self._iiot_publish(f"started:{target}")
        # Route through start_playback so IIoT cycles use the same
        # trajectory-action transport as manual GUI playback. progress_cb
        # will fire (0, total) on entry and push target waypoint 0 to SCADA;
        # subsequent waypoints publish via _on_traj_result.
        self.start_playback(
            seq,
            speed=self._iiot_default_speed,
            progress_cb=self._iiot_progress_cb,
            done_cb=self._iiot_finish_cb,
        )

    def _iiot_progress_cb(self, i: int, total: int):
        """Combined progress callback: status string + target waypoint pose."""
        self._iiot_publish(f"progress:{i}/{total}")
        # progress:i/total fires after wp[i-1] finished; the new target is wp[i]
        # if there is one. At i==total there's no next waypoint, just complete.
        if i < total:
            self._iiot_publish_target(i)

    def _iiot_publish_target(self, idx: int):
        """Publish seq.waypoints[idx] as JSON on iiot/target_waypoint."""
        seq = self._playback_seq
        if seq is None or idx < 0 or idx >= len(seq.waypoints):
            return
        wp = seq.waypoints[idx]
        payload = {
            "index": idx,
            "total": len(seq.waypoints),
            "joint_names": list(seq.joint_names),
            "positions": [float(p) for p in wp.positions],
            "gripper_width": wp.gripper_width,    # may be None
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._iiot_target_pub.publish(msg)

    def _iiot_finish_cb(self, ok: bool, reason: str):
        target = self._iiot_active_target
        self._iiot_active_target = None
        if ok:
            self._iiot_publish(f"complete:{target}")
        elif reason.startswith("Stopped"):
            self._iiot_publish(f"aborted:{target}")
        else:
            self._iiot_publish(f"error:{reason}")

    def _iiot_halt_cb(self, _msg: Empty):
        if self._playback_running:
            self.stop_playback()
        else:
            self._iiot_publish("ready")

    def _iiot_load_for_target(self, target: int) -> Optional["WaypointSequence"]:
        # If the GUI installed a resolver, use it. The resolver returns either
        # a Path to a YAML file or a WaypointSequence directly (for "use the
        # currently-loaded sequence" semantics).
        if self._iiot_resolve_target is None:
            return None
        try:
            resolved = self._iiot_resolve_target(target)
        except Exception:
            return None
        if resolved is None:
            return None
        if isinstance(resolved, WaypointSequence):
            return resolved
        try:
            with open(resolved) as f:
                data = yaml.safe_load(f) or {}
            return WaypointSequence.from_dict(data)
        except Exception:
            return None


class WaypointManager:
    """Public façade for the GUI — owns the rclpy node and a spin thread."""

    def __init__(self):
        self._cfg = load_config()
        self._sequence = WaypointSequence(arm_type=self._cfg.arm_type)
        self._counter = 0

        self._rclpy_owned = False
        if not rclpy.ok():
            rclpy.init()
            self._rclpy_owned = True

        self._node = _RosWorker()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:
            pass

    def shutdown(self):
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if self._rclpy_owned and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # ── Sequence operations ──────────────────────────────────────────────

    @property
    def sequence(self) -> WaypointSequence:
        return self._sequence

    def record_current(self, hold_time: float = 1.0) -> Optional[Waypoint]:
        snap = self._node.snapshot()
        if snap is None:
            return None
        # Joint names are pinned to the first recorded snapshot; the controller
        # only honours commands whose joint_names line up with the running arm,
        # so mixing snapshots from different layouts in one file is invalid.
        if not self._sequence.joint_names:
            self._sequence.joint_names = snap["names"]
        if snap["names"] != self._sequence.joint_names:
            # Re-order to match the recorded order, drop extras.
            name_to_pos = dict(zip(snap["names"], snap["positions"]))
            positions = [name_to_pos.get(n, 0.0) for n in self._sequence.joint_names]
        else:
            positions = list(snap["positions"])

        self._counter += 1
        wp = Waypoint(
            name=f"wp_{self._counter}",
            positions=positions,
            gripper_width=snap["gripper_width"],
            hold_time=float(hold_time),
        )
        self._sequence.waypoints.append(wp)
        return wp

    def remove_at(self, index: int):
        if 0 <= index < len(self._sequence.waypoints):
            del self._sequence.waypoints[index]

    def clear(self):
        self._sequence = WaypointSequence(arm_type=self._cfg.arm_type)
        self._counter = 0

    def update_hold_time(self, index: int, hold_time: float):
        if 0 <= index < len(self._sequence.waypoints):
            self._sequence.waypoints[index].hold_time = max(0.0, float(hold_time))

    # ── YAML I/O ─────────────────────────────────────────────────────────

    def save(self, path: str | Path):
        p = Path(path).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w") as f:
            yaml.safe_dump(self._sequence.to_dict(), f, sort_keys=False)

    def load(self, path: str | Path):
        p = Path(path).expanduser()
        with open(p) as f:
            data = yaml.safe_load(f) or {}
        self._sequence = WaypointSequence.from_dict(data)
        self._counter = len(self._sequence.waypoints)

    # ── Playback ─────────────────────────────────────────────────────────

    def play(
        self,
        speed: float,
        progress_cb: Callable[[int, int], None],
        done_cb: Callable[[bool, str], None],
    ):
        self._node.start_playback(self._sequence, speed, progress_cb, done_cb)

    def control_subscriber_count(self) -> int:
        return self._node.control_subscriber_count()

    def enable_arm(self, enable: bool, callback):
        """Call /enable_agx_arm. callback(ok: bool, msg: str) on completion."""
        self._node.call_enable_arm(enable, callback)

    def move_home(self, callback):
        """Call /move_home. callback(ok: bool, msg: str)."""
        self._node.call_move_home(callback)

    def traj_action_is_ready(self) -> bool:
        """True if /arm_controller/follow_joint_trajectory is currently available."""
        return self._node._traj_action.server_is_ready()

    def stop(self):
        self._node.stop_playback()

    def set_speed(self, speed: float):
        self._node.set_speed(speed)

    def set_paused(self, paused: bool):
        self._node.set_paused(paused)

    def is_playing(self) -> bool:
        return self._node.is_playing()

    # ── IIoT mode façade ─────────────────────────────────────────────────

    def configure_iiot(self, enabled: bool, resolve_target, default_speed: float = 1.0, event_cb=None):
        """Toggle IIoT device mode.

        resolve_target: callable target_id (int) → Path | WaypointSequence | None.
        Returning the live `manager.sequence` makes "play whatever is currently
        loaded" the no-config default.
        """
        self._node.configure_iiot(enabled, resolve_target, default_speed, event_cb)


# ---------------------------------------------------------------------------
# Standalone headless entry point
# ---------------------------------------------------------------------------

def main():
    """Run WaypointManager in IIoT device mode without the GUI.

    Reads waypoints_dir, target_map, and default_speed from gui_params.yaml
    (overlaid by gui_secrets.yaml). Exits cleanly on SIGINT / SIGTERM.
    """
    import signal

    cfg           = load_config()
    waypoints_dir = Path(cfg.iiot_waypoints_dir).expanduser()
    target_map    = cfg.iiot_target_map

    def resolve_target(target_id: int) -> Optional[Path]:
        filename = target_map.get(target_id)
        if filename is None:
            return None
        p = waypoints_dir / filename
        return p if p.is_file() else None

    manager = WaypointManager()
    manager.configure_iiot(
        enabled=True,
        resolve_target=resolve_target,
        default_speed=cfg.iiot_default_speed,
        event_cb=lambda s: print(f"[IIoT] {s}", flush=True),
    )

    print(f"[WaypointManager] IIoT mode active", flush=True)
    print(f"[WaypointManager] waypoints_dir : {waypoints_dir}", flush=True)
    print(f"[WaypointManager] target_map    : {target_map}", flush=True)

    running = True

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while running:
            time.sleep(0.5)
    finally:
        manager.shutdown()


if __name__ == "__main__":
    main()
