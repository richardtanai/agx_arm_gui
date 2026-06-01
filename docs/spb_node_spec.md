# Sparkplug B Bridge — System Specification

> **Current as of:** 2026-06  
> **Replaces:** all earlier `spb_node_spec`, `migration_to_isa`, `scada_integration`, and `mqtt_spb_link` docs  
> **Implementation:** `agx_arm_gui/spb_bridge_node.py`

---

## 1. Overview

`spb_bridge_node` is a ROS 2 node that exposes the robot arm's telemetry and a PackML command interface to any Sparkplug B–compliant MQTT host (Ignition SCADA, HiveMQ, custom clients).

It implements the full Sparkplug B device lifecycle:

```
NBIRTH → DBIRTH → DDATA ⇄ DCMD → NDEATH (LWT)
```

| Attribute | Value |
|---|---|
| Sparkplug namespace | `spBv1.0` |
| MQTT version | MQTTv3.1.1 |
| Library | Eclipse Tahu (`tahu.sparkplug_b`) |

---

## 2. ISA-95 / Sparkplug B Identity

| ISA-95 Level | Value | Sparkplug B field |
|---|---|---|
| Enterprise + Area + Site + Factory | `DMATDTS_DLSU_LS_MiniFactory` | Group ID |
| Cell | `agx_arm_bridge` | Edge Node ID |
| Unit | `piper_arm` | Device ID |

**Full DBIRTH topic:**
```
spBv1.0/DMATDTS_DLSU_LS_MiniFactory/DBIRTH/agx_arm_bridge/piper_arm
```

**Full SCADA tag path** (Ignition tag browser):
```
DMATDTS_DLSU_LS_MiniFactory / agx_arm_bridge / piper_arm / <metric>
```

Metric names are tag-path suffixes only — the device identity is encoded in the Sparkplug B topic, so no additional prefix is prepended to metric names.

All values are configurable in `config/gui_params.yaml` under `spb_bridge`.

---

## 3. MQTT Connection

| Parameter | Default | Source |
|---|---|---|
| Host | `localhost` | `gui_secrets.yaml → spb_bridge.local.host` |
| Port | `1883` | `gui_params.yaml → spb_bridge.local.port` |
| TLS | `false` | `gui_params.yaml → spb_bridge.local.use_tls` |
| Username / Password | *(empty)* | `gui_secrets.yaml` |
| Keep-alive | 60 s | hardcoded |
| Client ID | `agx_arm_bridge` | Edge Node ID |

For HiveMQ Cloud, set `broker_type: hivemq` in `gui_params.yaml` and fill `gui_secrets.yaml → spb_bridge.hivemq.*`.  
TLS connections use `ssl.CERT_REQUIRED` / `PROTOCOL_TLS_CLIENT`.

---

## 4. Topic Map

| Topic | Direction | QoS | Retain | When |
|---|---|---|---|---|
| `spBv1.0/…/NBIRTH/agx_arm_bridge` | Publish | 1 | No | On connect |
| `spBv1.0/…/NDEATH/agx_arm_bridge` | LWT | 1 | No | Broker auto-sends on ungraceful disconnect |
| `spBv1.0/…/DBIRTH/agx_arm_bridge/piper_arm` | Publish | 1 | No | On connect (after NBIRTH) |
| `spBv1.0/…/DDATA/agx_arm_bridge/piper_arm` | Publish | 0 | No | Timer-driven telemetry and state changes |
| `spBv1.0/…/DCMD/agx_arm_bridge/piper_arm` | Subscribe | 1 | — | Inbound SCADA commands |
| `spBv1.0/STATE/IgnitionPrimary` | Subscribe | 1 | — | Primary host liveness monitoring |

`…` = `DMATDTS_DLSU_LS_MiniFactory`

---

## 5. NBIRTH Payload

Published once per connect. Declares the node-level control metric.

| Metric | Type | Value | Description |
|---|---|---|---|
| `Node Control/Rebirth` | Boolean | `false` | SCADA writes `true` to force NBIRTH + DBIRTH re-publish |

---

## 6. DBIRTH — Full Tag Tree

Published once per connect immediately after NBIRTH. Defines every metric name and type so the SCADA tag browser is fully populated before any DDATA arrives.

### 6.1 PackML State — one-hot Booleans

Exactly one of these is `true` at any time.

| Metric | Type | Initial | Description |
|---|---|---|---|
| `Status/State/Current/Idle` | Boolean | `true` | Waiting for a Start command |
| `Status/State/Current/Execute` | Boolean | `false` | Motion in progress |
| `Status/State/Current/Complete` | Boolean | `false` | Cycle finished cleanly |
| `Status/State/Current/Aborted` | Boolean | `false` | Fault; alarms are active |
| `Status/Heartbeat` | Boolean | `false` | Toggles every 500 ms as a watchdog |

### 6.2 Commands — one-hot Booleans (SCADA-writable)

SCADA writes `true` to trigger a command. The bridge echoes `true` (acknowledgement), acts, then echoes `false` for **all four** command tags (one-hot reset).

| Metric | Type | Transition | Guard |
|---|---|---|---|
| `Cmd/CntrlCmd/Reset` | Boolean | Complete → Idle | State must be Complete |
| `Cmd/CntrlCmd/Start` | Boolean | Idle → Execute | State must be Idle; no active alarms |
| `Cmd/CntrlCmd/Stop` | Boolean | any → Idle | Always accepted; halts in-flight motion |
| `Cmd/CntrlCmd/Clear` | Boolean | Aborted → Idle | State must be Aborted; clears all alarms first |
| `Cmd/TargetID` | Int32 | *(stores value)* | Selects the waypoint sequence played on Start |

`Cmd/TargetID` is not a trigger command — it stores a value that is echoed back to SCADA on write.

### 6.3 Motion Telemetry

| Metric | Type | Unit | Description |
|---|---|---|---|
| `Motion/Joint/J1/Actual/Position` | Float | degrees | Joint 1 actual position |
| `Motion/Joint/J2/Actual/Position` | Float | degrees | Joint 2 actual position |
| `Motion/Joint/J3/Actual/Position` | Float | degrees | Joint 3 actual position |
| `Motion/Joint/J4/Actual/Position` | Float | degrees | Joint 4 actual position |
| `Motion/Joint/J5/Actual/Position` | Float | degrees | Joint 5 actual position |
| `Motion/Joint/J6/Actual/Position` | Float | degrees | Joint 6 actual position |
| `Gripper/Opening/Actual` | Float | 0–1 | Gripper opening fraction (0 = closed, 1 = fully open) |

Gripper fraction = `feedback_width_m / gripper_max_width_m`. Default `gripper_max_width_m = 0.1 m` (agx_gripper stroke).

### 6.4 Alarm Tree

One set of four sub-tags per alarm code. All codes are declared at DBIRTH so SCADA sees the full catalogue immediately.

| Metric | Type | Description |
|---|---|---|
| `Alarm/Active/{code}/State` | Int32 | `1` = Normal, `2` = Unacknowledged (active) |
| `Alarm/Active/{code}/Priority` | Int32 | NAMUR NE107 priority (see §9) |
| `Alarm/Active/{code}/Message` | String | Human-readable fault name |
| `Alarm/Active/{code}/OnsetMs` | Int64 | Unix epoch ms when alarm was raised |
| `Alarm/Summary/ActiveCount` | Int32 | Count of alarms currently in Unacknowledged state |

---

## 7. DDATA — Outbound Telemetry

Only changed metrics are included in each DDATA message (Report-by-Exception). Two timers run independently:

### 7.1 Telemetry Timer — 10 Hz

Fires every 100 ms. Applies deadbands; publishes a metric only when it has moved beyond the threshold. Every 5 seconds the full set is re-published regardless (stale-detection bypass).

| Metric group | Deadband | Full-republish |
|---|---|---|
| `Motion/Joint/J*/Actual/Position` | 0.1 ° | every 5 s |
| `Gripper/Opening/Actual` | 1 % of full stroke | every 5 s |

Source: `feedback/joint_states` (ROS subscription, 200 Hz from arm controller).  
Joint positions are converted from radians to degrees before publishing.

### 7.2 Heartbeat Timer — 2 Hz

`Status/Heartbeat` is toggled and published unconditionally every 500 ms.

### 7.3 State-change DDATA

Published immediately on every PackML state transition (not timer-driven). All four one-hot state tags are published in a single DDATA payload.

### 7.4 Alarm-change DDATA

Published immediately when an alarm is raised or cleared: `Alarm/Active/{code}/State`, `Alarm/Active/{code}/OnsetMs`, and `Alarm/Summary/ActiveCount`.

---

## 8. PackML State Machine

### States

| State | Meaning |
|---|---|
| **Idle** | Ready. Waiting for a Start command. |
| **Execute** | Motion in progress. |
| **Complete** | Cycle finished. Waiting for Reset to return to Idle. |
| **Aborted** | Fault state. One or more alarms are active. |

### Transitions

```
             Reset
 Complete ◄────────── ← Start ──── Idle
                                     ↑
                                     │ Stop (any state)
                                     │ Reset (Complete only)
                                     │ Clear (Aborted only)
                          Execute ───┘
                            │
                      motion complete → Complete
                      motion fault   → Aborted
                      timeout (60 s) → Aborted
```

| Command | From state | To state | Notes |
|---|---|---|---|
| `Start` | Idle | Execute | Triggers waypoint playback via `iiot/execute` |
| `Stop` | any | Idle | Halts in-flight motion immediately |
| `Reset` | Complete | Idle | — |
| `Clear` | Aborted | Idle | Clears all active alarms first |

### Command echo protocol

```
SCADA writes  Cmd/CntrlCmd/Start = true
   Bridge echoes                        → Start = true   (acknowledged)
   Bridge acts                          (state Idle → Execute)
   Bridge echoes                        → Reset=false, Start=false, Stop=false, Clear=false
```

The one-hot reset of all four command tags guarantees the SCADA tag browser is left clean regardless of which command was triggered.

---

## 9. Alarm Management

### Alarm codes

| Code | Priority | Message | Raised by |
|---|---|---|---|
| 7001 | 2 (High) | `MotionTimeout` | No `iiot/status` complete within 60 s |
| 7002 | 2 (High) | `MotionFailed` | `iiot/status` reports `aborted` or `error` |
| 7003 | 1 (Critical) | `PrimaryHostOffline` | Primary host STATE goes OFFLINE |
| 7004 | 1 (Critical) | `EStopAsserted` | *(reserved — not yet wired)* |
| 7005 | 2 (High) | `GripperPartLost` | *(reserved — not yet wired)* |
| 7006 | 4 (Maintenance) | `JointLimitApproach` | *(reserved — not yet wired)* |

**Priority mapping** (NAMUR NE107): 1 = Failure/Critical, 2 = High, 4 = Maintenance.

### Alarm lifecycle

```
Raise  → State = Unacknowledged (2), OnsetMs = now
Clear  → State = Normal (1)
```

Alarms are only raised once per fault (re-raise ignored if already Unacknowledged). `Clear` command clears all active alarms before transitioning Aborted → Idle.

---

## 10. Primary Host Monitoring → Safe State

The bridge subscribes to `spBv1.0/STATE/IgnitionPrimary` (configurable via `primary_host_id`).

| Event | Action |
|---|---|
| `ONLINE` received (first or edge) | Clear alarm 7003 |
| `OFFLINE` received (first or edge) | Raise alarm 7003 → halt motion → enter **Aborted** |

The bridge parses both JSON `{"online": true/false}` and plain-text `ONLINE`/`OFFLINE` payloads.

---

## 11. ROS Interface

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `feedback/joint_states` | `sensor_msgs/JointState` | Arm joint positions (radians); 200 Hz from arm controller |
| `feedback/gripper_status` | `agx_arm_msgs/GripperStatus` | Gripper opening width (metres) |

Joint names expected: `joint1` … `joint6`. Gripper telemetry is silently omitted if `agx_arm_msgs` is unavailable.

### Publications (IIoT bridge to WaypointManager)

| Topic | Type | When |
|---|---|---|
| `iiot/execute` | `std_msgs/Int32` | On `Cmd/CntrlCmd/Start` — carries `Cmd/TargetID` value |
| `iiot/halt` | `std_msgs/Empty` | On `Cmd/CntrlCmd/Stop` while a cycle is in flight |

### Subscription (WaypointManager to bridge)

| Topic | Type | Payloads |
|---|---|---|
| `iiot/status` | `std_msgs/String` | `ready` / `progress:i/t` / `complete:N` / `aborted:N` / `error:msg` |

| Status | Bridge action |
|---|---|
| `complete` | State Execute → Complete |
| `aborted` or `error` | Raise alarm 7002 → State Execute → Aborted |
| `ready`, `progress` | Ignored (informational) |

---

## 12. Configuration Reference

All fields below live in `config/gui_params.yaml` under `spb_bridge:`.  
Secrets (host, username, password) are overlaid from `config/gui_secrets.yaml`.

| Field | Default | Description |
|---|---|---|
| `group_id` | `DMATDTS_DLSU_LS_MiniFactory` | Sparkplug B Group ID |
| `edge_node_id` | `agx_arm_bridge` | Sparkplug B Edge Node ID |
| `device_id` | `piper_arm` | Sparkplug B Device ID |
| `primary_host_id` | `IgnitionPrimary` | Primary host name on `STATE/<id>` |
| `broker_type` | `local` | `local` or `hivemq` |
| `joint_deadband_deg` | `0.1` | Minimum joint change to trigger DDATA |
| `gripper_deadband` | `0.01` | Minimum gripper fraction change to trigger DDATA |
| `gripper_max_width_m` | `0.1` | Full stroke of the gripper in metres |
