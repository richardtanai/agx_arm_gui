"""Sparkplug B bridge node — Eclipse Tahu implementation.

Implements the AGX arm IIoT contract described in docs/iiot_node_design.md:

- Standard Handshake metrics (Station_Ready / Execute_Cmd / Processing /
  Job_Complete / Current_Step / Fault_Code) with PropertySet metadata so the
  primary application can decode integer codes into human-readable strings.
- PackML-aligned state machine (Idle / Execute / Complete / Aborted /
  Stopped), exposed alongside the legacy Status/Overall tag.
- Last Will & Testament — NDEATH is registered before connect().
- Primary Host Monitoring — subscribes to STATE/<primary_host_id>; on OFFLINE
  the robot enters a Safe State (Halt + Status/PackML = Aborted).
- Report-by-Exception (RBE) — joint and EE telemetry only emit metrics that
  changed beyond a configurable deadband, plus a periodic full publish that
  acts as a heartbeat for stale-detection on the SCADA side.
"""

import json
import math
import ssl
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Empty, String
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

import paho.mqtt.client as mqtt

import tahu.sparkplug_b_pb2 as spb_pb2
from tahu.sparkplug_b import (
    MetricDataType,
    addMetric,
    getNodeDeathPayload,
    getNodeBirthPayload,
    getDeviceBirthPayload,
    getDdataPayload,
)

from .config_loader import load_config

SPB_NAMESPACE = "spBv1.0"
JOINT_RATE_HZ = 10.0
RBE_FULL_PUBLISH_PERIOD_S = 5.0  # heartbeat-style full telemetry republish

# PackML states (ISA-TR88) — used in Status/PackML
PACKML_IDLE     = "Idle"
PACKML_EXECUTE  = "Execute"
PACKML_COMPLETE = "Complete"
PACKML_ABORTED  = "Aborted"
PACKML_STOPPED  = "Stopped"

# Fault code dictionary — published as PropertySet metadata on Fault_Code so
# the primary application can render the human-readable name. Integer keys are
# stringified because PropertySet keys are strings.
FAULT_CODES = {
    0:   "OK",
    100: "ConnectionLost",
    200: "MotionFailed",
    201: "MotionTimeout",
    300: "PrimaryHostOffline",
    400: "EStopAsserted",
    500: "InvalidCommand",
}

# Current step lookup — describes the sub-state values published in
# Current_Step. These mirror the 4-phase handshake.
STEP_CODES = {
    0: "Idle",
    1: "Triggered",
    2: "Acknowledged",
    3: "Executing",
    4: "Complete",
    9: "Aborted",
}


class SpbBridgeNode(Node):
    """ROS 2 node bridging robot telemetry to MQTT via Sparkplug B."""

    def __init__(self):
        super().__init__("spb_bridge_node")

        node_cfg = load_config()
        broker   = node_cfg.active_broker()

        # ── Sparkplug B identity ────────────────────────────────────────
        self.declare_parameter("group_id",         node_cfg.spb_group_id)
        self.declare_parameter("edge_node_id",     node_cfg.spb_edge_node_id)
        self.declare_parameter("device_id",        node_cfg.spb_device_id)
        self.declare_parameter("primary_host_id",  node_cfg.primary_host_id)

        # ── MQTT broker ─────────────────────────────────────────────────
        self.declare_parameter("mqtt_host",     broker.host)
        self.declare_parameter("mqtt_port",     broker.port)
        self.declare_parameter("mqtt_username", broker.username)
        self.declare_parameter("mqtt_password", broker.password)
        self.declare_parameter("use_tls",       broker.use_tls)

        # ── ROS / TF ────────────────────────────────────────────────────
        self.declare_parameter("base_frame",    node_cfg.base_frame)
        self.declare_parameter("ee_frame",      node_cfg.ee_frame)
        self.declare_parameter("sim_mode",      False)

        # ── RBE deadbands (units already in degrees / metres) ──────────
        self.declare_parameter("joint_deadband_deg", node_cfg.joint_deadband_deg)
        self.declare_parameter("ee_deadband_m",      node_cfg.ee_deadband_m)
        self.declare_parameter("ee_deadband_deg",    node_cfg.ee_deadband_deg)
        self.declare_parameter("gripper_max_width_m", node_cfg.gripper_max_width_m)
        self.declare_parameter("gripper_deadband",    node_cfg.gripper_deadband)

        # Resolve parameters
        self._group_id        = self.get_parameter("group_id").value
        self._edge_node_id    = self.get_parameter("edge_node_id").value
        self._device_id       = self.get_parameter("device_id").value
        self._primary_host_id = self.get_parameter("primary_host_id").value

        self._mqtt_host  = self.get_parameter("mqtt_host").value
        self._mqtt_port  = self.get_parameter("mqtt_port").value
        self._mqtt_user  = self.get_parameter("mqtt_username").value
        self._mqtt_pass  = self.get_parameter("mqtt_password").value
        self._use_tls    = self.get_parameter("use_tls").value

        self._base_frame = self.get_parameter("base_frame").value
        self._ee_frame   = self.get_parameter("ee_frame").value
        self._sim_mode   = self.get_parameter("sim_mode").value

        self._joint_db_deg     = float(self.get_parameter("joint_deadband_deg").value)
        self._ee_db_m          = float(self.get_parameter("ee_deadband_m").value)
        self._ee_db_deg        = float(self.get_parameter("ee_deadband_deg").value)
        self._gripper_max_m    = float(self.get_parameter("gripper_max_width_m").value)
        self._gripper_deadband = float(self.get_parameter("gripper_deadband").value)

        # ── Topics ─────────────────────────────────────────────────────
        _ns = SPB_NAMESPACE
        _g  = self._group_id
        _n  = self._edge_node_id
        _d  = self._device_id
        self._NBIRTH_TOPIC = f"{_ns}/{_g}/NBIRTH/{_n}"
        self._NDEATH_TOPIC = f"{_ns}/{_g}/NDEATH/{_n}"
        self._DBIRTH_TOPIC = f"{_ns}/{_g}/DBIRTH/{_n}/{_d}"
        self._DDEATH_TOPIC = f"{_ns}/{_g}/DDEATH/{_n}/{_d}"
        self._DDATA_TOPIC  = f"{_ns}/{_g}/DDATA/{_n}/{_d}"
        self._DCMD_TOPIC   = f"{_ns}/{_g}/DCMD/{_n}/{_d}"
        # Primary Host monitoring topic — Sparkplug B v3 puts this under STATE/<host_id>.
        self._STATE_TOPIC  = f"{_ns}/STATE/{self._primary_host_id}"

        # ── Robot state ────────────────────────────────────────────────
        self._status_mode: str = "Idle"
        self._packml: str      = PACKML_IDLE
        self._busy: bool       = False
        self._done: bool       = False
        self._heartbeat: bool  = False
        self._connected: bool  = False
        self._station_ready: bool = True   # local permissive — set False on E-Stop / fault
        self._processing: bool    = False
        self._job_complete: bool  = False  # latch — only cleared by SCADA write
        self._current_step: int   = 0
        self._fault_code: int     = 0
        self._primary_online: bool = False  # set True once we see STATE=ONLINE

        # 4-phase handshake state
        self._trigger_active: bool = False
        self._target_id: int = 0
        self._done_set_time: float = 0.0

        # RBE caches — last-published values, keyed by metric name
        self._last_published: dict = {}
        self._last_full_publish: float = 0.0

        # ── TF2 ────────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Joint state cache — slotted by joint name so we are immune to
        # ordering differences between publishers (arm controller appends
        # gripper/hand after the arm joints; waypoint playback may not).
        self._arm_joint_names: tuple = tuple(f"joint{i}" for i in range(1, 7))
        self._joint_positions: list = [0.0] * 6
        self._joint_msgs_seen: int = 0
        self._joint_name_warn_last: float = 0.0

        # Gripper: width in metres, normalised to [0, 1] for SCADA.
        self._gripper_width_m: Optional[float] = None

        # Target waypoint pose, mirrored from the WaypointManager during IIoT
        # playback. SCADA sees these as Positions/Target/* alongside the actual
        # Positions/Joints / Positions/Gripper readings.
        self._target_joint_positions: dict = {}    # joint_name → radians
        self._target_gripper_width_m: Optional[float] = None
        self._target_index: int = -1
        self._tf_warn_last: float = 0.0
        self._joint_warn_last: float = 0.0

        # ── MQTT client ────────────────────────────────────────────────
        self._mqtt = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            client_id=self._edge_node_id,
            protocol=mqtt.MQTTv311,
        )
        if self._use_tls:
            self._mqtt.tls_set(cert_reqs=ssl.CERT_REQUIRED,
                               tls_version=ssl.PROTOCOL_TLS_CLIENT)
        if self._mqtt_user:
            self._mqtt.username_pw_set(self._mqtt_user, self._mqtt_pass)

        self._mqtt.on_connect    = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect
        self._mqtt.on_message    = self._on_mqtt_message

        # LWT — node dies if we drop the connection without a clean disconnect
        lwt_payload = getNodeDeathPayload()
        self._mqtt.will_set(
            self._NDEATH_TOPIC,
            lwt_payload.SerializeToString(),
            qos=1,
            retain=False,
        )

        self._mqtt.connect_async(self._mqtt_host, self._mqtt_port, keepalive=60)
        self._mqtt.loop_start()

        self.get_logger().info(
            f"SPB Bridge — connecting to {self._mqtt_host}:{self._mqtt_port} "
            f"(TLS={'on' if self._use_tls else 'off'}) | "
            f"{self._group_id}/{self._edge_node_id}/{self._device_id} "
            f"| primary_host_id={self._primary_host_id}"
        )

        # ── ROS subscriptions ──────────────────────────────────────────
        # Telemetry must reflect the arm's *actual* state, never commanded
        # waypoints. Subscribing to control/joint_states as well makes the
        # callback flip between commanded and achieved poses on every cycle —
        # SCADA then sees the joint values "jump". feedback/ only.
        self.create_subscription(JointState, "feedback/joint_states", self._joint_cb, 10)

        # GripperStatus lives in agx_arm_msgs — import lazily so the bridge
        # still loads if the overlay isn't sourced (gripper telemetry is
        # optional and simply stays at None / 0.0 in that case).
        try:
            from agx_arm_msgs.msg import GripperStatus
            self.create_subscription(
                GripperStatus, "feedback/gripper_status", self._gripper_cb, 10
            )
        except Exception as exc:
            self.get_logger().warn(
                f"GripperStatus unavailable, Positions/Gripper will stay at 0: {exc}"
            )

        # ── IIoT bridge (SPB ↔ WaypointManager) ────────────────────────
        self._iiot_execute_pub = self.create_publisher(Int32, "iiot/execute", 10)
        self._iiot_halt_pub    = self.create_publisher(Empty, "iiot/halt",    10)
        self.create_subscription(String, "iiot/status", self._iiot_status_cb, 10)
        # Target waypoint pose published by the WaypointManager at each step.
        self.create_subscription(String, "iiot/target_waypoint",
                                 self._iiot_target_cb, 10)
        self._iiot_in_flight: bool = False

        # ── Timers ─────────────────────────────────────────────────────
        self.create_timer(1.0 / JOINT_RATE_HZ, self._telemetry_timer_cb)
        self.create_timer(0.5, self._heartbeat_timer_cb)
        self.create_timer(2.0, self._done_clear_timer_cb)

    # ------------------------------------------------------------------
    # MQTT callbacks
    # ------------------------------------------------------------------

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.get_logger().error(f"Broker rejected connection, rc={rc}")
            return

        self.get_logger().info("MQTT connected — publishing NBIRTH / DBIRTH")
        self._connected = True
        self._last_published.clear()      # birth resets RBE state on the SCADA side
        self._last_full_publish = 0.0

        client.subscribe(self._DCMD_TOPIC, qos=1)
        client.subscribe(self._STATE_TOPIC, qos=1)

        self._publish_nbirth(client)
        self._publish_dbirth(client)

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._connected = False
        self.get_logger().warn(f"MQTT disconnected (rc={rc}), paho will reconnect")

    def _on_mqtt_message(self, client, userdata, msg):
        if msg.topic == self._DCMD_TOPIC:
            self._handle_dcmd(msg.payload)
        elif msg.topic == self._STATE_TOPIC:
            self._handle_primary_state(msg.payload)

    # ------------------------------------------------------------------
    # Birth publishing
    # ------------------------------------------------------------------

    def _publish_nbirth(self, client):
        payload = getNodeBirthPayload()
        addMetric(payload, "Node Control/Rebirth", None, MetricDataType.Boolean, False)
        client.publish(self._NBIRTH_TOPIC, payload.SerializeToString(), qos=1)

    def _publish_dbirth(self, client):
        payload = getDeviceBirthPayload()

        # ── Status (legacy, kept for back-compat) ──────────────────────
        addMetric(payload, "Status/Overall",   None, MetricDataType.String,  self._status_mode)
        addMetric(payload, "Status/Busy",      None, MetricDataType.Boolean, self._busy)
        addMetric(payload, "Status/Done",      None, MetricDataType.Boolean, self._done)
        addMetric(payload, "Status/Heartbeat", None, MetricDataType.Boolean, self._heartbeat)
        # ── Status (PackML / ISA-TR88) ─────────────────────────────────
        addMetric(payload, "Status/PackML",    None, MetricDataType.String,  self._packml)

        # ── Standard Handshake (ISA-95 station contract) ───────────────
        addMetric(payload, "Handshake/Station_Ready", None, MetricDataType.Boolean, self._station_ready)
        addMetric(payload, "Handshake/Execute_Cmd",   None, MetricDataType.Boolean, False)
        addMetric(payload, "Handshake/Processing",    None, MetricDataType.Boolean, self._processing)
        addMetric(payload, "Handshake/Job_Complete",  None, MetricDataType.Boolean, self._job_complete)

        # Current_Step / Fault_Code with PropertySet metadata for decode
        step_metric = addMetric(payload, "Handshake/Current_Step", None,
                                MetricDataType.Int32, self._current_step)
        _attach_lookup_properties(step_metric, STEP_CODES)

        fault_metric = addMetric(payload, "Handshake/Fault_Code", None,
                                 MetricDataType.Int32, self._fault_code)
        _attach_lookup_properties(fault_metric, FAULT_CODES)

        # ── Joint telemetry (placeholders) ─────────────────────────────
        for i in range(1, 7):
            addMetric(payload, f"Positions/Joints/J{i}", None, MetricDataType.Float, 0.0)
        # ── EE pose (placeholders) ─────────────────────────────────────
        for axis in ("X", "Y", "Z", "Roll", "Pitch", "Yaw"):
            addMetric(payload, f"Positions/EE/{axis}", None, MetricDataType.Float, 0.0)
        # ── Gripper opening, normalised [0=closed, 1=fully open] ───────
        addMetric(payload, "Positions/Gripper", None, MetricDataType.Float, 0.0)
        # ── Target waypoint pose (commanded, not measured). Updated each
        #    time the WaypointManager moves on to a new waypoint during an
        #    IIoT cycle. Stays at last value between cycles. ─────────────
        for i in range(1, 7):
            addMetric(payload, f"Positions/Target/J{i}", None, MetricDataType.Float, 0.0)
        addMetric(payload, "Positions/Target/Gripper", None, MetricDataType.Float, 0.0)

        # ── Command metrics (initial readable state) ──────────────────
        addMetric(payload, "Commands/Trigger",      None, MetricDataType.Boolean, False)
        addMetric(payload, "Commands/TargetID",     None, MetricDataType.Int32,   0)
        addMetric(payload, "Commands/Halt",         None, MetricDataType.Boolean, False)
        # SCADA writes False here to clear the Job_Complete latch — Transactional Integrity
        addMetric(payload, "Commands/JobComplete_Ack", None, MetricDataType.Boolean, False)

        client.publish(self._DBIRTH_TOPIC, payload.SerializeToString(), qos=1)

    # ------------------------------------------------------------------
    # DDATA helpers
    # ------------------------------------------------------------------

    def _publish_ddata(self, metrics: dict):
        """metrics: {name: (MetricDataType, value)} — published unconditionally."""
        if not self._connected or not metrics:
            return
        payload = getDdataPayload()
        for name, (dtype, value) in metrics.items():
            addMetric(payload, name, None, dtype, value)
            self._last_published[name] = value
        self._mqtt.publish(self._DDATA_TOPIC, payload.SerializeToString(), qos=0)

    def _publish_rbe(self, metrics: dict, deadbands: dict):
        """Publish only metrics whose value moved more than its deadband.

        deadbands: {metric_name: float}. Missing key → any change publishes.
        """
        changed: dict = {}
        for name, (dtype, value) in metrics.items():
            prev = self._last_published.get(name)
            if prev is None:
                changed[name] = (dtype, value)
                continue
            db = deadbands.get(name, 0.0)
            try:
                if abs(float(value) - float(prev)) > db:
                    changed[name] = (dtype, value)
            except (TypeError, ValueError):
                if value != prev:
                    changed[name] = (dtype, value)
        if changed:
            self._publish_ddata(changed)

    # ------------------------------------------------------------------
    # ROS callbacks & timers
    # ------------------------------------------------------------------

    def _joint_cb(self, msg: JointState):
        # Slot positions into J1..J6 by name. Falls back to index order only
        # if msg.name is empty — the controller, MoveIt, and waypoint_manager
        # all set msg.name correctly, so the fallback should never trigger.
        if msg.name:
            name_to_pos = dict(zip(msg.name, msg.position))
            missing = [n for n in self._arm_joint_names if n not in name_to_pos]
            if missing:
                now = time.monotonic()
                if (now - self._joint_name_warn_last) >= 5.0:
                    self._joint_name_warn_last = now
                    self.get_logger().warn(
                        f"joint_states missing expected names {missing}; "
                        f"received {list(msg.name)}"
                    )
                return
            self._joint_positions = [
                float(name_to_pos[n]) for n in self._arm_joint_names
            ]
        else:
            self._joint_positions = list(msg.position[:6])

        if self._joint_msgs_seen == 0:
            self.get_logger().info(
                f"First feedback/joint_states received "
                f"(names={list(msg.name)[:6]}) — joint telemetry will publish"
            )
        self._joint_msgs_seen += 1

    def _gripper_cb(self, msg):
        # GripperStatus.width is in metres. Cache it; normalisation happens
        # in the telemetry timer so SCADA always sees a fresh fraction.
        try:
            self._gripper_width_m = float(msg.width)
        except (AttributeError, TypeError, ValueError):
            self._gripper_width_m = None

    def _telemetry_timer_cb(self):
        if not self._connected:
            return

        now = time.monotonic()
        full_publish = (now - self._last_full_publish) >= RBE_FULL_PUBLISH_PERIOD_S

        metrics: dict = {}
        deadbands: dict = {}

        for i, pos in enumerate(self._joint_positions, start=1):
            name = f"Positions/Joints/J{i}"
            metrics[name] = (MetricDataType.Float, math.degrees(float(pos)))
            deadbands[name] = self._joint_db_deg

        # Gripper opening, normalised to [0, 1]. If we've never received a
        # status yet, publish 0.0 (matches the DBIRTH placeholder).
        if self._gripper_max_m > 0.0:
            raw = self._gripper_width_m if self._gripper_width_m is not None else 0.0
            frac = max(0.0, min(1.0, raw / self._gripper_max_m))
            metrics["Positions/Gripper"] = (MetricDataType.Float, frac)
            deadbands["Positions/Gripper"] = self._gripper_deadband

        # Target waypoint pose. Joint values arrive in radians; gripper width
        # in metres. Same units / normalisation as the actual pose metrics.
        for i, name in enumerate(self._arm_joint_names, start=1):
            metric_name = f"Positions/Target/J{i}"
            tval_rad = float(self._target_joint_positions.get(name, 0.0))
            metrics[metric_name] = (MetricDataType.Float, math.degrees(tval_rad))
            deadbands[metric_name] = self._joint_db_deg
        if self._gripper_max_m > 0.0:
            traw = (self._target_gripper_width_m
                    if self._target_gripper_width_m is not None else 0.0)
            tfrac = max(0.0, min(1.0, traw / self._gripper_max_m))
            metrics["Positions/Target/Gripper"] = (MetricDataType.Float, tfrac)
            deadbands["Positions/Target/Gripper"] = self._gripper_deadband

        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                self._base_frame, self._ee_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            roll, pitch, yaw = _quat_to_rpy(q.x, q.y, q.z, q.w)
            for axis, value, db in (
                ("X",     float(t.x),               self._ee_db_m),
                ("Y",     float(t.y),               self._ee_db_m),
                ("Z",     float(t.z),               self._ee_db_m),
                ("Roll",  math.degrees(roll),       self._ee_db_deg),
                ("Pitch", math.degrees(pitch),      self._ee_db_deg),
                ("Yaw",   math.degrees(yaw),        self._ee_db_deg),
            ):
                name = f"Positions/EE/{axis}"
                metrics[name] = (MetricDataType.Float, value)
                deadbands[name] = db
        except Exception as exc:
            if (now - self._tf_warn_last) >= 5.0:
                self._tf_warn_last = now
                self.get_logger().warn(
                    f"EE pose unavailable: TF '{self._base_frame}' → "
                    f"'{self._ee_frame}' lookup failed ({exc}). "
                    f"Check that robot_state_publisher is running and that "
                    f"ee_frame names a link in the URDF."
                )

        if self._joint_msgs_seen == 0 and (now - self._joint_warn_last) >= 5.0:
            self._joint_warn_last = now
            self.get_logger().warn(
                "No joint_states received yet on feedback/joint_states — "
                "joint telemetry will publish zeros. Is the arm controller "
                "running and Enabled?"
            )

        if full_publish:
            self._publish_ddata(metrics)
            self._last_full_publish = now
        else:
            self._publish_rbe(metrics, deadbands)

    def _heartbeat_timer_cb(self):
        self._heartbeat = not self._heartbeat
        # Heartbeat is its own metric — RBE doesn't apply, this is the watchdog.
        self._publish_ddata({"Status/Heartbeat": (MetricDataType.Boolean, self._heartbeat)})

    def _done_clear_timer_cb(self):
        if self._done and (time.monotonic() - self._done_set_time) >= 2.0:
            self._done = False
            self._publish_ddata({"Status/Done": (MetricDataType.Boolean, False)})

    # ------------------------------------------------------------------
    # 4-phase handshake & DCMD handling
    # ------------------------------------------------------------------

    def _handle_dcmd(self, raw: bytes):
        try:
            payload = spb_pb2.Payload()
            payload.ParseFromString(raw)
        except Exception as exc:
            self.get_logger().error(f"Failed to parse DCMD: {exc}")
            return

        # Surface the whole DCMD message so we can see exactly what SCADA wrote.
        summary = ", ".join(_metric_repr(m) for m in payload.metrics) or "(no metrics)"
        self.get_logger().info(
            f"DCMD received [{summary}] | state: trigger_active={self._trigger_active}, "
            f"target_id={self._target_id}, job_complete={self._job_complete}, "
            f"in_flight={self._iiot_in_flight}, primary_online={self._primary_online}"
        )

        # Echo every recognised command back as DDATA so Ignition's tag
        # browser display matches what SCADA last wrote. Without this, writes
        # snap back to the DBIRTH default because the edge node owns the
        # value in Sparkplug and we never confirmed receipt.
        echoes: dict = {}

        for metric in payload.metrics:
            name = metric.name
            if name == "Commands/Halt":
                halt = bool(metric.boolean_value)
                echoes[name] = (MetricDataType.Boolean, halt)
                if halt:
                    self.get_logger().info("[Halt] HALT received — stopping motion")
                    self._on_halt()

            elif name == "Commands/TargetID":
                old = self._target_id
                self._target_id = int(metric.int_value)
                echoes[name] = (MetricDataType.Int32, self._target_id)
                self.get_logger().info(
                    f"[TargetID] {old} → {self._target_id}"
                )

            elif name == "Commands/Trigger":
                trig = bool(metric.boolean_value)
                echoes[name] = (MetricDataType.Boolean, trig)
                if trig and not self._trigger_active:
                    self.get_logger().info(
                        f"[Phase 1] Trigger rising edge — TargetID={self._target_id}, "
                        f"entering acknowledge phase"
                    )
                    self._trigger_active = True
                    self._set_current_step(1)  # Triggered
                    self._phase2_acknowledge()
                elif not trig and self._trigger_active:
                    self.get_logger().info(
                        "[Phase 3] Trigger falling edge — dispatching motion"
                    )
                    self._phase3_execute()
                elif trig and self._trigger_active:
                    self.get_logger().warn(
                        "[Trigger] Ignored: Trigger=true but cycle already active. "
                        "SCADA must drop Trigger to false before re-arming."
                    )
                else:
                    # Trigger=false while not active — typical idle state, debug only.
                    self.get_logger().debug(
                        "[Trigger] Ignored: Trigger=false and no cycle in flight (idle)."
                    )

            elif name == "Commands/JobComplete_Ack":
                ack = bool(metric.boolean_value)
                echoes[name] = (MetricDataType.Boolean, ack)
                if not ack and self._job_complete:
                    self._job_complete = False
                    self._publish_ddata({
                        "Handshake/Job_Complete": (MetricDataType.Boolean, False),
                    })
                    self.get_logger().info(
                        "[Phase 4] JobComplete_Ack=false — latch cleared, ready for next cycle"
                    )
                elif not ack and not self._job_complete:
                    self.get_logger().debug(
                        "[Ack] JobComplete_Ack=false but no latch to clear (idle)."
                    )
                else:
                    self.get_logger().debug(
                        f"[Ack] JobComplete_Ack=true received (no-op, latch clears on falling edge)."
                    )

        if echoes:
            self._publish_ddata(echoes)

    def _phase2_acknowledge(self):
        self._busy = True
        self._processing = True
        self._packml = PACKML_EXECUTE
        self._status_mode = "Fake" if self._sim_mode else "Physical"
        self._set_current_step(2)
        self._publish_ddata({
            "Status/Busy":              (MetricDataType.Boolean, True),
            "Status/Overall":           (MetricDataType.String,  self._status_mode),
            "Status/PackML":            (MetricDataType.String,  self._packml),
            "Handshake/Processing":     (MetricDataType.Boolean, True),
        })
        self.get_logger().info(
            f"[Phase 2] Acknowledged → Busy=true, PackML={self._packml}, "
            f"Step=2. Awaiting Trigger=false to start motion."
        )

    def _phase3_execute(self):
        """Hand the motion off to the WaypointManager via iiot/execute.

        Phase 4 is triggered asynchronously when iiot/status reports complete.
        If no manager is listening, we fall through to a fault after a timeout.
        """
        self._set_current_step(3)
        self._iiot_in_flight = True
        msg = Int32()
        msg.data = int(self._target_id)

        sub_count = self._iiot_execute_pub.get_subscription_count()
        if sub_count == 0:
            self.get_logger().error(
                f"[Phase 3] iiot/execute has NO subscribers — the WaypointManager "
                f"is not listening. Is IIoT Device Mode ticked in the GUI? "
                f"Publishing target={self._target_id} anyway; will time out in 60 s."
            )
        else:
            self.get_logger().info(
                f"[Phase 3] iiot/execute → target={self._target_id} "
                f"({sub_count} subscriber{'s' if sub_count != 1 else ''}). "
                f"Awaiting iiot/status complete:{self._target_id} from WaypointManager "
                f"(timeout 60 s)."
            )

        self._iiot_execute_pub.publish(msg)
        # Watchdog — if nothing comes back inside 60 s, mark MotionTimeout.
        threading.Timer(60.0, self._iiot_timeout_check).start()

    def _iiot_timeout_check(self):
        if self._iiot_in_flight:
            self._iiot_in_flight = False
            self.get_logger().error(
                "[Phase 3] MotionTimeout — no iiot/status complete received in 60 s. "
                "Check that IIoT Device Mode is enabled, the target YAML exists, and "
                "the arm controller/MoveIt is responding."
            )
            self._set_error(201, "MotionTimeout")

    def _iiot_target_cb(self, msg: String):
        """Cache the target waypoint pose published by the WaypointManager.

        Telemetry timer reads from these caches and emits Positions/Target/*.
        """
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"[Target] failed to parse iiot/target_waypoint: {exc}")
            return

        names = data.get("joint_names") or []
        positions = data.get("positions") or []
        if len(names) == len(positions):
            self._target_joint_positions = {
                str(n): float(p) for n, p in zip(names, positions)
            }
        gw = data.get("gripper_width")
        self._target_gripper_width_m = float(gw) if gw is not None else None
        self._target_index = int(data.get("index", -1))
        self.get_logger().info(
            f"[Target] waypoint {self._target_index + 1}/{data.get('total', '?')} "
            f"cached ({len(names)} joints, "
            f"gripper_width={self._target_gripper_width_m})"
        )

    def _iiot_status_cb(self, msg: String):
        text = msg.data
        if not text:
            return
        # Format: "started:N" / "progress:i/total" / "complete:N" / "aborted:N" / "error:msg" / "ready"
        head, _, _ = text.partition(":")

        # 'ready' / 'progress' arrive frequently; log them at debug to avoid spam.
        if head in ("ready", "progress"):
            self.get_logger().debug(f"[iiot/status] {text}")
        else:
            self.get_logger().info(
                f"[iiot/status] {text}  (in_flight={self._iiot_in_flight})"
            )

        if head == "complete" and self._iiot_in_flight:
            self._iiot_in_flight = False
            self._phase4_complete()
        elif head == "aborted" and self._iiot_in_flight:
            self._iiot_in_flight = False
            self.get_logger().warn(f"[iiot/status] aborted — halting cycle")
            self._on_halt()
        elif head == "error" and self._iiot_in_flight:
            self._iiot_in_flight = False
            self.get_logger().error(f"[iiot/status] error from WaypointManager: {text}")
            self._set_error(200, f"MotionFailed:{text[6:]}")

    def _phase4_complete(self):
        self._busy = False
        self._done = True
        self._processing = False
        self._job_complete = True   # latched until SCADA Ack
        self._done_set_time = time.monotonic()
        self._trigger_active = False
        self._packml = PACKML_COMPLETE
        self._status_mode = "Idle"
        self._set_current_step(4)
        self._publish_ddata({
            "Status/Busy":             (MetricDataType.Boolean, False),
            "Status/Done":             (MetricDataType.Boolean, True),
            "Status/Overall":          (MetricDataType.String,  "Idle"),
            "Status/PackML":           (MetricDataType.String,  self._packml),
            "Handshake/Processing":    (MetricDataType.Boolean, False),
            "Handshake/Job_Complete":  (MetricDataType.Boolean, True),
        })
        self.get_logger().info(
            "[Phase 4] Motion complete — Job_Complete=true (latched), "
            "Step=4, PackML=Complete. SCADA must write JobComplete_Ack=false to clear."
        )

        # PackML transitions Complete → Idle automatically once Done clears.
        threading.Timer(2.0, self._return_to_idle).start()

    def _return_to_idle(self):
        if self._packml == PACKML_COMPLETE:
            self._packml = PACKML_IDLE
            self._publish_ddata({"Status/PackML": (MetricDataType.String, PACKML_IDLE)})

    def _set_error(self, code: int, label: str):
        self._busy = False
        self._processing = False
        self._trigger_active = False
        self._packml = PACKML_ABORTED
        self._status_mode = "Error"
        self._fault_code = code
        self._set_current_step(9)
        self._publish_ddata({
            "Status/Busy":           (MetricDataType.Boolean, False),
            "Status/Overall":        (MetricDataType.String,  "Error"),
            "Status/PackML":         (MetricDataType.String,  PACKML_ABORTED),
            "Handshake/Processing":  (MetricDataType.Boolean, False),
            "Handshake/Fault_Code":  (MetricDataType.Int32,   code),
        })
        self.get_logger().error(f"Status → Error ({code}={label})")

    def _on_halt(self):
        self._busy = False
        self._done = False
        self._processing = False
        self._trigger_active = False
        self._iiot_in_flight = False
        self._packml = PACKML_STOPPED
        self._status_mode = "Idle"
        self._set_current_step(0)
        # Tell the manager to stop in case a playback is mid-flight.
        self._iiot_halt_pub.publish(Empty())
        self._publish_ddata({
            "Status/Busy":           (MetricDataType.Boolean, False),
            "Status/Overall":        (MetricDataType.String,  "Idle"),
            "Status/PackML":         (MetricDataType.String,  PACKML_STOPPED),
            "Handshake/Processing":  (MetricDataType.Boolean, False),
        })

    def _set_current_step(self, step: int):
        if step != self._current_step:
            self._current_step = step
            # Always publish — Current_Step is discrete state, not analogue.
            self._publish_ddata({
                "Handshake/Current_Step": (MetricDataType.Int32, step),
            })

    # ------------------------------------------------------------------
    # Primary Host monitoring → Safe State
    # ------------------------------------------------------------------

    def _handle_primary_state(self, raw: bytes):
        """Sparkplug B v3 publishes JSON: {online: bool, timestamp: int, bdSeq: int}.
        Older v2.2 brokers publish a plain ASCII string ('ONLINE' / 'OFFLINE').
        Accept both."""
        text = raw.decode("utf-8", errors="replace").strip()
        online: Optional[bool] = None
        try:
            obj = json.loads(text)
            if isinstance(obj, dict) and "online" in obj:
                online = bool(obj["online"])
        except Exception:
            pass
        if online is None:
            online = text.upper() == "ONLINE"

        if online and not self._primary_online:
            self._primary_online = True
            self.get_logger().info(f"Primary host '{self._primary_host_id}' ONLINE")
        elif not online and self._primary_online:
            self._primary_online = False
            self.get_logger().warn(
                f"Primary host '{self._primary_host_id}' OFFLINE — entering Safe State"
            )
            self._enter_safe_state()

    def _enter_safe_state(self):
        """Halt motion and report fault code 300 (PrimaryHostOffline)."""
        self._on_halt()
        self._fault_code = 300
        self._publish_ddata({
            "Handshake/Fault_Code": (MetricDataType.Int32, 300),
        })

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        try:
            self._mqtt.disconnect()
        except Exception:
            pass
        try:
            self._mqtt.loop_stop()
        except Exception:
            pass
        super().destroy_node()


# ---------------------------------------------------------------------------
# PropertySet helpers
# ---------------------------------------------------------------------------

def _attach_lookup_properties(metric, lookup: dict):
    """Attach an Engineering-style PropertySet to a metric.

    We store two properties per metric:
    - "Documentation": JSON-encoded {int: name} dictionary, parseable by Ignition
      transformations or any custom client.
    - "EngUnit": empty string (kept for spec parity; can be set by callers).
    """
    ps = metric.properties
    ps.keys.append("Documentation")
    pv = ps.values.add()
    pv.type = 12  # String
    pv.string_value = json.dumps({str(k): v for k, v in lookup.items()})

    ps.keys.append("EngUnit")
    pv = ps.values.add()
    pv.type = 12
    pv.string_value = ""


# ---------------------------------------------------------------------------
# Sparkplug metric debug helper
# ---------------------------------------------------------------------------

def _metric_repr(metric) -> str:
    """Render a Sparkplug metric as 'name=value' for log lines. Used to surface
    exactly what SCADA wrote on every DCMD so we can debug stuck handshakes."""
    name = metric.name or "(no-name)"
    if metric.HasField("boolean_value"):
        return f"{name}={metric.boolean_value}"
    if metric.HasField("int_value"):
        return f"{name}={metric.int_value}"
    if metric.HasField("long_value"):
        return f"{name}={metric.long_value}"
    if metric.HasField("float_value"):
        return f"{name}={metric.float_value}"
    if metric.HasField("double_value"):
        return f"{name}={metric.double_value}"
    if metric.HasField("string_value"):
        return f"{name}={metric.string_value!r}"
    return f"{name}=?"


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _quat_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SpbBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
