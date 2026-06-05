"""Sparkplug B bridge node — ISA-95 identity.

Implements the partial spec defined in docs/migration_to_isa.md:

- ISA-95 identity:  GID=DMATDTS_DLSU_LS_MiniFactory, Node=agx_arm_bridge, Device=piper_arm.
- Metrics published directly under the device (no additional prefix).
- PackML state machine driven by SCADA Boolean writes to Cmd/CntrlCmd/<name>:
      Cmd/CntrlCmd/Reset, /Start, /Stop, /Clear — write True to trigger
      Status/State/Current/<name> — one-hot Booleans: /Idle, /Execute, /Complete, /Aborted
- Cmd/TargetID (Int32) selects the waypoint sequence played on Start.
- Sparse alarm tree under Alarm/Active/<code>/ (codes 7001..7006).
- Motion telemetry under Motion/Joint/J<n>/Actual/Position; gripper opening under
  Gripper/Opening/Actual.
- LWT (NDEATH) registered before connect; Primary Host monitoring drives the
  bridge into Safe State (Aborted + alarm 7003) when STATE/<host_id> goes
  offline.
"""

import json
import math
import ssl
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, Empty, String

_TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

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
METRIC_PREFIX = ""  # identity is already encoded in Group/Node/Device

JOINT_RATE_HZ = 10.0
RBE_FULL_PUBLISH_PERIOD_S = 5.0
MOTION_TIMEOUT_S = 60.0

# One-hot Boolean state names — Status/State/Current
STATE_IDLE     = "Idle"
STATE_EXECUTE  = "Execute"
STATE_COMPLETE = "Complete"
STATE_ABORTED  = "Aborted"

# One-hot Boolean sub-tags for Status/State/Current (published as DDATA on transition)
_STATE_BOOL_TAGS = {
    STATE_IDLE:     "Status/State/Current/Idle",
    STATE_EXECUTE:  "Status/State/Current/Execute",
    STATE_COMPLETE: "Status/State/Current/Complete",
    STATE_ABORTED:  "Status/State/Current/Aborted",
}

# Cmd/CntrlCmd Boolean sub-tag names — write True to trigger
CMD_RESET = "Reset"
CMD_START = "Start"
CMD_STOP  = "Stop"
CMD_CLEAR = "Clear"

# Boolean sub-tags for Cmd/CntrlCmd (SCADA writes True to trigger; bridge echoes False)
_CMD_BOOL_TAGS = {
    CMD_RESET: "Cmd/CntrlCmd/Reset",
    CMD_START: "Cmd/CntrlCmd/Start",
    CMD_STOP:  "Cmd/CntrlCmd/Stop",
    CMD_CLEAR: "Cmd/CntrlCmd/Clear",
}

# Reverse lookup used in _handle_dcmd: full metric name → CMD code
_CMD_BOOL_NAMES: dict = {}  # populated after _m() is defined below

# ISA-18.2 alarm lifecycle subset used by the bridge.
ALARM_NORMAL = 1
ALARM_UNACK  = 2

# Static alarm definitions: code → (priority, message). Priority follows
# NAMUR NE107 mapping (1=Failure/Critical .. 4=Maintenance).
ALARM_DEFINITIONS = {
    7001: (2, "MotionTimeout"),
    7002: (2, "MotionFailed"),
    7003: (1, "PrimaryHostOffline"),
    7004: (1, "EStopAsserted"),
    7005: (2, "GripperPartLost"),
    7006: (4, "JointLimitApproach"),
}

# Default cycle target_id when SCADA writes Cmd/CntrlCmd=Start. The migration
# spec exposes no SCADA-writable target selector, so the bridge always plays
# the currently-loaded waypoint sequence (target_map[0] in gui_params.yaml).
DEFAULT_TARGET_ID = 0


def _m(suffix: str) -> str:
    return f"{METRIC_PREFIX}/{suffix}" if METRIC_PREFIX else suffix


# Populate the reverse-lookup now that _m() is available.
_CMD_BOOL_NAMES = {_m(suffix): code for code, suffix in _CMD_BOOL_TAGS.items()}


class SpbBridgeNode(Node):
    """ROS 2 node bridging robot telemetry to MQTT via Sparkplug B (ISA-95)."""

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

        # ── Robot params ────────────────────────────────────────────────
        self.declare_parameter("sim_mode",            False)
        self.declare_parameter("joint_deadband_deg",  node_cfg.joint_deadband_deg)
        self.declare_parameter("gripper_max_width_m", node_cfg.gripper_max_width_m)
        self.declare_parameter("gripper_deadband",    node_cfg.gripper_deadband)

        # Resolve parameters
        self._group_id        = self.get_parameter("group_id").value
        self._edge_node_id    = self.get_parameter("edge_node_id").value
        self._device_id       = self.get_parameter("device_id").value
        self._primary_host_id = self.get_parameter("primary_host_id").value

        self._mqtt_host = self.get_parameter("mqtt_host").value
        self._mqtt_port = self.get_parameter("mqtt_port").value
        self._mqtt_user = self.get_parameter("mqtt_username").value
        self._mqtt_pass = self.get_parameter("mqtt_password").value
        self._use_tls   = self.get_parameter("use_tls").value

        self._sim_mode      = self.get_parameter("sim_mode").value
        self._joint_db_deg  = float(self.get_parameter("joint_deadband_deg").value)
        self._gripper_max_m = float(self.get_parameter("gripper_max_width_m").value)
        self._gripper_db    = float(self.get_parameter("gripper_deadband").value)

        # ── Sparkplug topics ──────────────────────────────────────────
        _ns, _g = SPB_NAMESPACE, self._group_id
        _n, _d  = self._edge_node_id, self._device_id
        self._NBIRTH_TOPIC = f"{_ns}/{_g}/NBIRTH/{_n}"
        self._NDEATH_TOPIC = f"{_ns}/{_g}/NDEATH/{_n}"
        self._DBIRTH_TOPIC = f"{_ns}/{_g}/DBIRTH/{_n}/{_d}"
        self._DDEATH_TOPIC = f"{_ns}/{_g}/DDEATH/{_n}/{_d}"
        self._DDATA_TOPIC  = f"{_ns}/{_g}/DDATA/{_n}/{_d}"
        self._DCMD_TOPIC   = f"{_ns}/{_g}/DCMD/{_n}/{_d}"
        self._STATE_TOPIC  = f"{_ns}/STATE/{self._primary_host_id}"

        # ── Runtime state ──────────────────────────────────────────────
        self._state: str = STATE_IDLE
        self._heartbeat: bool = False
        self._connected: bool = False
        self._primary_seen: bool = False
        self._primary_online: bool = False
        self._iiot_in_flight: bool = False
        self._target_id: int = DEFAULT_TARGET_ID
        # OMAC PackML command authority: False=Manual/local panel, True=Auto/SCADA
        self._remote_mode: bool = False

        self._alarm_states: dict = {c: ALARM_NORMAL for c in ALARM_DEFINITIONS}
        self._alarm_onsets: dict = {c: 0 for c in ALARM_DEFINITIONS}

        # Telemetry caches
        self._joint_positions: list = [0.0] * 6
        self._gripper_width_m: Optional[float] = None
        self._arm_joint_names = tuple(f"joint{i}" for i in range(1, 7))
        self._joint_msgs_seen = 0
        self._joint_name_warn_last = 0.0
        self._joint_warn_last = 0.0

        # RBE caches
        self._last_published: dict = {}
        self._last_full_publish: float = 0.0

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

        # LWT registered before connect so the broker publishes NDEATH on our behalf
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
            f"SPB Bridge (ISA-95) — connecting to {self._mqtt_host}:{self._mqtt_port} "
            f"(TLS={'on' if self._use_tls else 'off'}) | "
            f"{self._group_id}/{self._edge_node_id}/{self._device_id} | "
            f"prefix={METRIC_PREFIX} | primary_host_id={self._primary_host_id}"
        )

        # ── ROS subscriptions ──────────────────────────────────────────
        self.create_subscription(JointState, "feedback/joint_states", self._joint_cb, 10)

        try:
            from agx_arm_msgs.msg import GripperStatus
            self.create_subscription(
                GripperStatus, "feedback/gripper_status", self._gripper_cb, 10
            )
        except Exception as exc:
            self.get_logger().warn(
                f"GripperStatus unavailable, Gripper/Opening/Actual will stay at 0: {exc}"
            )

        # ── IIoT bridge (internal — SPB ↔ WaypointManager) ────────────
        self._iiot_execute_pub = self.create_publisher(Int32, "iiot/execute", 10)
        self._iiot_halt_pub    = self.create_publisher(Empty, "iiot/halt",    10)
        self.create_subscription(String, "iiot/status", self._iiot_status_cb, 10)

        # ── Panel integration ───────────────────────────────────────────
        # arm/state: TRANSIENT_LOCAL so panel_node gets the current state
        # immediately on subscribe (even if no transition has happened yet).
        self._arm_state_pub = self.create_publisher(
            String, "arm/state", _TRANSIENT_LOCAL_QOS
        )
        self.create_subscription(String, "panel/cmd",   self._panel_cmd_cb,   10)
        self.create_subscription(Bool,   "panel/estop", self._panel_estop_cb, 10)
        self.create_subscription(String, "panel/mode",  self._panel_mode_cb,  10)

        # Seed the TRANSIENT_LOCAL topic with the initial state so panel_node
        # gets it on its first subscription without waiting for a transition.
        _init_state_msg = String()
        _init_state_msg.data = self._state
        self._arm_state_pub.publish(_init_state_msg)

        # ── Timers ─────────────────────────────────────────────────────
        self.create_timer(1.0 / JOINT_RATE_HZ, self._telemetry_timer_cb)
        self.create_timer(0.5, self._heartbeat_timer_cb)

    # ------------------------------------------------------------------
    # MQTT callbacks
    # ------------------------------------------------------------------

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.get_logger().error(f"Broker rejected connection, rc={rc}")
            return
        self.get_logger().info("MQTT connected — publishing NBIRTH / DBIRTH")
        self._connected = True
        self._last_published.clear()
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

        # Status — one-hot Booleans; exactly one is True at any time
        for state_code, suffix in _STATE_BOOL_TAGS.items():
            addMetric(payload, _m(suffix), None,
                      MetricDataType.Boolean, self._state == state_code)
        addMetric(payload, _m("Status/Heartbeat"), None,
                  MetricDataType.Boolean, self._heartbeat)
        # OMAC PackML command authority (ISA-TR88.00.02 §5.4)
        # Remote=False → Manual/local panel owns Start; Remote=True → SCADA owns Start
        addMetric(payload, _m("Status/Remote"),   None,
                  MetricDataType.Boolean, self._remote_mode)
        # UnitMode: 1=Production (SCADA), 3=Manual (panel)
        addMetric(payload, _m("Status/UnitMode"), None,
                  MetricDataType.Int32, 1 if self._remote_mode else 3)

        # Cmd — Boolean sub-tags. SCADA writes True to trigger; bridge echoes
        # False after acting. TargetID retains its value across cycles.
        for suffix in _CMD_BOOL_TAGS.values():
            addMetric(payload, _m(suffix), None, MetricDataType.Boolean, False)
        addMetric(payload, _m("Cmd/TargetID"), None,
                  MetricDataType.Int32, self._target_id)

        # Motion — joint actual positions in degrees
        for i in range(1, 7):
            addMetric(payload, _m(f"Motion/Joint/J{i}/Actual/Position"), None,
                      MetricDataType.Float, 0.0)

        # Gripper opening normalised [0=closed .. 1=fully open]
        addMetric(payload, _m("Gripper/Opening/Actual"), None,
                  MetricDataType.Float, 0.0)

        # Alarm tree — all known codes declared at Normal. State transitions
        # happen via DDATA; this lets SCADA discover the full alarm set from
        # DBIRTH without needing a Rebirth on every alarm change.
        for code, (priority, message) in ALARM_DEFINITIONS.items():
            addMetric(payload, _m(f"Alarm/Active/{code}/State"), None,
                      MetricDataType.Int32, self._alarm_states[code])
            addMetric(payload, _m(f"Alarm/Active/{code}/Priority"), None,
                      MetricDataType.Int32, priority)
            addMetric(payload, _m(f"Alarm/Active/{code}/Message"), None,
                      MetricDataType.String, message)
            addMetric(payload, _m(f"Alarm/Active/{code}/OnsetMs"), None,
                      MetricDataType.Int64, self._alarm_onsets[code])
        addMetric(payload, _m("Alarm/Summary/ActiveCount"), None,
                  MetricDataType.Int32, self._active_alarm_count())

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
    # ROS callbacks & telemetry
    # ------------------------------------------------------------------

    def _joint_cb(self, msg: JointState):
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
            self._joint_positions = [float(name_to_pos[n]) for n in self._arm_joint_names]
        else:
            self._joint_positions = list(msg.position[:6])

        if self._joint_msgs_seen == 0:
            self.get_logger().info(
                f"First feedback/joint_states received (names={list(msg.name)[:6]})"
            )
        self._joint_msgs_seen += 1

    def _gripper_cb(self, msg):
        try:
            self._gripper_width_m = float(msg.width)
        except (AttributeError, TypeError, ValueError):
            self._gripper_width_m = None

    def _telemetry_timer_cb(self):
        if not self._connected:
            return
        now = time.monotonic()
        full = (now - self._last_full_publish) >= RBE_FULL_PUBLISH_PERIOD_S

        metrics: dict = {}
        deadbands: dict = {}

        for i, pos in enumerate(self._joint_positions, start=1):
            name = _m(f"Motion/Joint/J{i}/Actual/Position")
            metrics[name] = (MetricDataType.Float, math.degrees(float(pos)))
            deadbands[name] = self._joint_db_deg

        if self._gripper_max_m > 0.0:
            raw = self._gripper_width_m if self._gripper_width_m is not None else 0.0
            frac = max(0.0, min(1.0, raw / self._gripper_max_m))
            name = _m("Gripper/Opening/Actual")
            metrics[name] = (MetricDataType.Float, frac)
            deadbands[name] = self._gripper_db

        if self._joint_msgs_seen == 0 and (now - self._joint_warn_last) >= 5.0:
            self._joint_warn_last = now
            self.get_logger().warn(
                "No joint_states yet on feedback/joint_states — Motion/Joint/* will publish zeros."
            )

        if full:
            self._publish_ddata(metrics)
            self._last_full_publish = now
        else:
            self._publish_rbe(metrics, deadbands)

    def _heartbeat_timer_cb(self):
        self._heartbeat = not self._heartbeat
        self._publish_ddata({
            _m("Status/Heartbeat"): (MetricDataType.Boolean, self._heartbeat),
        })

    # ------------------------------------------------------------------
    # DCMD handling — Cmd/CntrlCmd is the only writable metric
    # ------------------------------------------------------------------

    def _handle_dcmd(self, raw: bytes):
        try:
            payload = spb_pb2.Payload()
            payload.ParseFromString(raw)
        except Exception as exc:
            self.get_logger().error(f"Failed to parse DCMD: {exc}")
            return

        target_name = _m("Cmd/TargetID")
        for metric in payload.metrics:
            name = metric.name
            if name in _CMD_BOOL_NAMES:
                if not metric.boolean_value:
                    # Only act on True writes; ignore the False echo-back.
                    continue
                code = _CMD_BOOL_NAMES[name]
                self.get_logger().info(
                    f"DCMD Cmd/CntrlCmd/{code}=True | "
                    f"state={self._state} | "
                    f"target_id={self._target_id}"
                )
                # Echo True — confirms command received.
                self._publish_ddata({name: (MetricDataType.Boolean, True)})
                self._execute_cntrl_cmd(code)
                # Echo False for all cmd tags — one-hot reset so SCADA tag browser is clean.
                self._publish_ddata({
                    _m(suffix): (MetricDataType.Boolean, False)
                    for suffix in _CMD_BOOL_TAGS.values()
                })

            elif name == target_name:
                try:
                    new_id = int(metric.int_value)
                except Exception:
                    self.get_logger().error("DCMD: Cmd/TargetID not Int32")
                    continue
                old = self._target_id
                self._target_id = new_id
                self.get_logger().info(f"DCMD Cmd/TargetID: {old} → {new_id}")
                # Echo back so the SCADA tag browser reflects the stored value.
                self._publish_ddata({target_name: (MetricDataType.Int32, new_id)})

            else:
                self.get_logger().warn(f"DCMD ignored (unknown metric): {name}")

    def _execute_cntrl_cmd(self, code: str, source: str = "scada"):
        if code == CMD_RESET:
            if self._state == STATE_COMPLETE:
                self._set_state(STATE_IDLE)
            else:
                self.get_logger().warn(
                    f"Reset ignored: state must be Complete (got {self._state})"
                )
            return

        if code == CMD_START:
            # OMAC PackML command-authority gating.
            # Start is only valid from the source that owns the current UnitMode.
            # Stop/Reset/Clear are always accepted from any source (PackML mandatory).
            if source == "scada" and not self._remote_mode:
                self.get_logger().warn(
                    "SCADA Start rejected: UnitMode=Manual — panel has command authority"
                )
                return
            if source == "panel" and self._remote_mode:
                self.get_logger().warn(
                    "Panel Start rejected: UnitMode=Auto — SCADA has command authority"
                )
                return
            if self._state != STATE_IDLE:
                self.get_logger().warn(
                    f"Start ignored: state must be Idle (got {self._state})"
                )
                return
            if self._active_alarm_count() > 0:
                self.get_logger().warn("Start ignored: active alarms must be cleared first")
                return
            self._start_cycle()
            return

        if code == CMD_STOP:
            # Stop interrupts execution and returns to Idle. Per the migration
            # spec's 4-state subset there is no Stopped state — we collapse the
            # full PackML Stopping→Stopped→Resetting→Idle chain to a single hop.
            self._halt_motion()
            self._set_state(STATE_IDLE)
            return

        if code == CMD_CLEAR:
            if self._state != STATE_ABORTED:
                self.get_logger().warn(
                    f"Clear ignored: state must be Aborted (got {self._state})"
                )
                return
            for c in list(self._alarm_states.keys()):
                if self._alarm_states[c] != ALARM_NORMAL:
                    self._clear_alarm(c)
            self._set_state(STATE_IDLE)
            return

        self.get_logger().warn(f"DCMD: unsupported Cmd/CntrlCmd code {code}")

    # ------------------------------------------------------------------
    # PackML state transitions & cycle lifecycle
    # ------------------------------------------------------------------

    def _set_state(self, new_state: str):
        if new_state == self._state:
            return
        self.get_logger().info(f"State: {self._state} → {new_state}")
        self._state = new_state
        self._publish_ddata({
            _m(suffix): (MetricDataType.Boolean, new_state == state_code)
            for state_code, suffix in _STATE_BOOL_TAGS.items()
        })
        # Notify panel_node (TRANSIENT_LOCAL — last value retained for late joiners)
        _msg = String()
        _msg.data = new_state
        self._arm_state_pub.publish(_msg)

    def _start_cycle(self):
        self._set_state(STATE_EXECUTE)
        self._iiot_in_flight = True
        msg = Int32()
        msg.data = int(self._target_id)
        if self._iiot_execute_pub.get_subscription_count() == 0:
            self.get_logger().error(
                f"iiot/execute has NO subscribers — WaypointManager not listening. "
                f"target={self._target_id} will time out in {MOTION_TIMEOUT_S:.0f} s."
            )
        else:
            self.get_logger().info(f"iiot/execute → target={self._target_id}")
        self._iiot_execute_pub.publish(msg)
        threading.Timer(MOTION_TIMEOUT_S, self._motion_timeout_check).start()

    def _motion_timeout_check(self):
        if self._iiot_in_flight:
            self._iiot_in_flight = False
            self.get_logger().error("MotionTimeout — no iiot/status complete in time")
            self._set_aborted(7001)

    def _halt_motion(self):
        if self._iiot_in_flight:
            self._iiot_halt_pub.publish(Empty())
            self._iiot_in_flight = False

    def _iiot_status_cb(self, msg: String):
        text = msg.data
        if not text:
            return
        head, _, _ = text.partition(":")
        if head in ("ready", "progress"):
            return
        self.get_logger().info(f"[iiot/status] {text} (in_flight={self._iiot_in_flight})")
        if head == "complete" and self._iiot_in_flight:
            self._iiot_in_flight = False
            self._set_state(STATE_COMPLETE)
        elif head in ("aborted", "error") and self._iiot_in_flight:
            self._iiot_in_flight = False
            self._set_aborted(7002)

    # ------------------------------------------------------------------
    # Alarm management
    # ------------------------------------------------------------------

    def _raise_alarm(self, code: int):
        if code not in ALARM_DEFINITIONS:
            self.get_logger().error(f"Unknown alarm code {code}")
            return
        if self._alarm_states[code] == ALARM_UNACK:
            return
        priority, message = ALARM_DEFINITIONS[code]
        now_ms = int(time.time() * 1000)
        self._alarm_states[code] = ALARM_UNACK
        self._alarm_onsets[code] = now_ms
        self.get_logger().warn(
            f"ALARM {code} ({message}) priority={priority} raised at {now_ms}"
        )
        self._publish_ddata({
            _m(f"Alarm/Active/{code}/State"):   (MetricDataType.Int32, ALARM_UNACK),
            _m(f"Alarm/Active/{code}/OnsetMs"): (MetricDataType.Int64, now_ms),
            _m("Alarm/Summary/ActiveCount"):    (MetricDataType.Int32, self._active_alarm_count()),
        })

    def _clear_alarm(self, code: int):
        if code not in ALARM_DEFINITIONS or self._alarm_states[code] == ALARM_NORMAL:
            return
        self._alarm_states[code] = ALARM_NORMAL
        _, message = ALARM_DEFINITIONS[code]
        self.get_logger().info(f"ALARM {code} ({message}) cleared")
        self._publish_ddata({
            _m(f"Alarm/Active/{code}/State"): (MetricDataType.Int32, ALARM_NORMAL),
            _m("Alarm/Summary/ActiveCount"):  (MetricDataType.Int32, self._active_alarm_count()),
        })

    def _active_alarm_count(self) -> int:
        return sum(1 for s in self._alarm_states.values() if s != ALARM_NORMAL)

    def _set_aborted(self, alarm_code: int):
        self._halt_motion()
        self._raise_alarm(alarm_code)
        self._set_state(STATE_ABORTED)

    # ------------------------------------------------------------------
    # Primary Host monitoring → Safe State
    # ------------------------------------------------------------------

    def _handle_primary_state(self, raw: bytes):
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

        # First message: always act on it, even if it agrees with the default.
        # Subsequent messages: only act on edges.
        if self._primary_seen and online == self._primary_online:
            return
        self._primary_seen = True

        if online:
            self._primary_online = True
            self.get_logger().info(f"Primary host '{self._primary_host_id}' ONLINE")
            self._clear_alarm(7003)
        else:
            self._primary_online = False
            self.get_logger().warn(
                f"Primary host '{self._primary_host_id}' OFFLINE — entering Safe State"
            )
            self._set_aborted(7003)

    # ------------------------------------------------------------------
    # Panel integration callbacks
    # ------------------------------------------------------------------

    def _panel_cmd_cb(self, msg: String):
        """panel/cmd subscriber — panel_node publishes 'start'|'stop'|'reset'|'clear'."""
        raw = msg.data.strip()
        # Normalise to the CMD_* constants (first letter upper-case)
        code = raw.title()
        if code not in (CMD_START, CMD_STOP, CMD_RESET, CMD_CLEAR):
            self.get_logger().warn(f"panel/cmd: unknown command '{raw}'")
            return
        self.get_logger().info(f"Panel cmd: {code} (state={self._state})")
        self._execute_cntrl_cmd(code, source="panel")

    def _panel_estop_cb(self, msg: Bool):
        """panel/estop subscriber — True = E-Stop asserted, False = released."""
        if msg.data:
            self.get_logger().warn("panel/estop: E-Stop asserted → Aborted + alarm 7004")
            self._set_aborted(7004)
        else:
            self.get_logger().info("panel/estop: E-Stop released")
            if self._alarm_states.get(7004) != ALARM_NORMAL:
                self._clear_alarm(7004)
            # Return to Idle only if E-Stop was the only active alarm and we're Aborted
            if self._state == STATE_ABORTED and self._active_alarm_count() == 0:
                self._set_state(STATE_IDLE)

    def _panel_mode_cb(self, msg: String):
        """panel/mode subscriber — 'manual' | 'auto'."""
        remote = msg.data.strip().lower() == "auto"
        self._set_mode(remote)

    def _set_mode(self, remote: bool):
        """Switch between Manual (panel authority) and Auto (SCADA authority)."""
        if remote == self._remote_mode:
            return
        self._remote_mode = remote
        self.get_logger().info(
            f"UnitMode → {'Auto/Production (SCADA authority)' if remote else 'Manual (panel authority)'}"
        )
        self._publish_ddata({
            _m("Status/Remote"):   (MetricDataType.Boolean, remote),
            _m("Status/UnitMode"): (MetricDataType.Int32,   1 if remote else 3),
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
