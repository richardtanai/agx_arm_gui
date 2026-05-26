# Sparkplug B Node Specification — agx_arm_bridge

## 1. Overview

The `spb_bridge_node` is a ROS 2 node that exposes robot telemetry and a command interface to any Sparkplug B–compliant MQTT host (Ignition SCADA, HiveMQ, custom clients). It implements the full Sparkplug B device lifecycle: NBIRTH → DBIRTH → DDATA → DCMD → NDEATH.

**Implementation library:** Eclipse Tahu (`tahu.sparkplug_b_pb2`, `tahu.sparkplug_b`)
**Protocol version:** `spBv1.0`
**MQTT version:** MQTTv3.1.1

---

## 2. Identity

| Parameter      | Default value     | ROS parameter     |
|----------------|-------------------|-------------------|
| Group ID       | `DMAT`            | `group_id`        |
| Edge Node ID   | `agx_arm_bridge`  | `edge_node_id`    |
| Device ID      | `piper_arm`       | `device_id`       |

Configured in `config/gui_params.yaml` under `spb_bridge`. Override at launch with `--ros-args -p group_id:=<value>`.

---

## 3. MQTT Connection

| Parameter   | Default       | ROS parameter   |
|-------------|---------------|-----------------|
| Host        | `localhost`   | `mqtt_host`     |
| Port        | `1883`        | `mqtt_port`     |
| Username    | *(empty)*     | `mqtt_username` |
| Password    | *(empty)*     | `mqtt_password` |
| TLS         | `false`       | `use_tls`       |
| Keep-alive  | 60 s          | —               |
| Client ID   | `agx_arm_bridge` (edge node ID) | — |

TLS uses `ssl.CERT_REQUIRED` with `PROTOCOL_TLS_CLIENT`. Credentials are optional; leave empty for anonymous brokers.

---

## 4. Topic Map

All topics follow the pattern `spBv1.0/{group_id}/{msg_type}/{edge_node_id}[/{device_id}]`.

| Topic | Direction | QoS | Retain | Trigger |
|---|---|---|---|---|
| `spBv1.0/DMAT/NBIRTH/agx_arm_bridge` | Publish | 1 | No | On MQTT connect |
| `spBv1.0/DMAT/NDEATH/agx_arm_bridge` | Publish (LWT) | 1 | No | Broker auto-sends on ungraceful disconnect |
| `spBv1.0/DMAT/DBIRTH/agx_arm_bridge/piper_arm` | Publish | 1 | No | On MQTT connect (after NBIRTH) |
| `spBv1.0/DMAT/DDEATH/agx_arm_bridge/piper_arm` | — | — | — | Defined but not explicitly published |
| `spBv1.0/DMAT/DDATA/agx_arm_bridge/piper_arm` | Publish | 0 | No | Continuously (telemetry) or on state change |
| `spBv1.0/DMAT/DCMD/agx_arm_bridge/piper_arm` | Subscribe | 1 | — | Inbound commands from SCADA/PLC |

---

## 5. NBIRTH Payload

Published once on connect. Declares the node-level metrics to the primary application.

| Metric Name | Type | Initial Value | Description |
|---|---|---|---|
| `Node Control/Rebirth` | Boolean | `false` | Writing `true` triggers a re-publish of NBIRTH + DBIRTH |

---

## 6. DBIRTH Payload

Published once on connect, immediately after NBIRTH. Defines the complete device metric schema — types and initial values — so the primary application can build the tag tree before any DDATA arrives.

### Status Metrics

| Metric Name | Type | Initial Value | Description |
|---|---|---|---|
| `Status/Overall` | String | `"Idle"` | Current operating mode. Values: `Idle`, `Physical`, `Fake`, `Error` |
| `Status/Busy` | Boolean | `false` | `true` while a motion trajectory is executing |
| `Status/Done` | Boolean | `false` | Pulses `true` for 2 s after successful motion completion |
| `Status/Heartbeat` | Boolean | `false` | Toggles every 500 ms — used as a watchdog |

### Joint Telemetry

| Metric Name | Type | Initial Value | Unit |
|---|---|---|---|
| `Positions/Joints/J1` | Float | `0.0` | Degrees |
| `Positions/Joints/J2` | Float | `0.0` | Degrees |
| `Positions/Joints/J3` | Float | `0.0` | Degrees |
| `Positions/Joints/J4` | Float | `0.0` | Degrees |
| `Positions/Joints/J5` | Float | `0.0` | Degrees |
| `Positions/Joints/J6` | Float | `0.0` | Degrees |

### End-Effector Pose

TF lookup from `base_link` → `gripper_link`. Frames are configurable via `base_frame` / `ee_frame` ROS parameters.

| Metric Name | Type | Initial Value | Unit |
|---|---|---|---|
| `Positions/EE/X` | Float | `0.0` | Metres |
| `Positions/EE/Y` | Float | `0.0` | Metres |
| `Positions/EE/Z` | Float | `0.0` | Metres |
| `Positions/EE/Roll` | Float | `0.0` | Degrees |
| `Positions/EE/Pitch` | Float | `0.0` | Degrees |
| `Positions/EE/Yaw` | Float | `0.0` | Degrees |

### Command Metrics (initial readable state)

| Metric Name | Type | Initial Value |
|---|---|---|
| `Commands/Trigger` | Boolean | `false` |
| `Commands/TargetID` | Int32 | `0` |
| `Commands/Halt` | Boolean | `false` |

---

## 7. DDATA — Outbound Telemetry

DDATA messages carry only the metrics that changed (delta encoding). Two independent timers drive outbound DDATA:

### 7a. Telemetry Timer (10 Hz)

Fires every 100 ms. Publishes joint positions and, if a TF lookup succeeds within 50 ms, the EE pose. If the TF lookup fails the EE metrics are omitted for that cycle.

| Metric | Rate | Notes |
|---|---|---|
| `Positions/Joints/J1–J6` | 10 Hz | Source: `feedback/joint_states` or `control/joint_states` (whichever arrives last) |
| `Positions/EE/X,Y,Z` | 10 Hz | Omitted if TF unavailable |
| `Positions/EE/Roll,Pitch,Yaw` | 10 Hz | Omitted if TF unavailable |

### 7b. Heartbeat Timer (2 Hz)

Publishes `Status/Heartbeat` (Boolean toggle) every 500 ms independently of the telemetry timer.

### 7c. State-change DDATA

Published immediately on any robot state transition (not timer-driven):

| Event | Metrics published |
|---|---|
| Motion acknowledged (Phase 2) | `Status/Busy = true`, `Status/Overall = "Physical"` (or `"Fake"` in sim mode) |
| Motion complete (Phase 4) | `Status/Busy = false`, `Status/Done = true`, `Status/Overall = "Idle"` |
| Done pulse cleared (2 s after completion) | `Status/Done = false` |
| Halt received | `Status/Busy = false`, `Status/Overall = "Idle"` |
| Motion error | `Status/Busy = false`, `Status/Overall = "Error"` |

---

## 8. DCMD — Inbound Commands

The node subscribes to the DCMD topic and processes three command metrics. Commands can be sent individually or combined in a single payload.

| Metric Name | Type | Behaviour |
|---|---|---|
| `Commands/TargetID` | Int32 | Stores the target pose index. Set this before asserting Trigger. |
| `Commands/Trigger` | Boolean | `true` → starts the 4-phase handshake. `false` → clears trigger and begins execution (Phase 3). |
| `Commands/Halt` | Boolean | `true` → immediately aborts the active motion and sets `Status/Overall = "Idle"`. |

---

## 9. 4-Phase Handshake (Motion Sequence)

```
PLC / SCADA                        Bridge Node
     │                                   │
     │── DCMD: TargetID = N ────────────►│  (stores target index)
     │── DCMD: Trigger   = true ────────►│  Phase 1 — Request
     │                                   │── DDATA: Busy=true, Overall="Physical"  ◄─ Phase 2 — Acknowledge
     │── DCMD: Trigger   = false ───────►│  Phase 3 — Execute (motion starts)
     │                                   │  ... motion running ...
     │                                   │── DDATA: Busy=false, Done=true, Overall="Idle"  ◄─ Phase 4 — Complete
     │                                   │── DDATA: Done=false  (2 s later, auto-clear)
```

**Abort path:** Send `Commands/Halt = true` at any point during Phase 2–4.

---

## 10. Status/Overall State Machine

```
          ┌─────────────────────┐
          │        Idle         │◄──── startup / halt / phase 4 complete
          └─────────┬───────────┘
                    │ Trigger received
                    ▼
          ┌─────────────────────┐
          │  Physical / Fake    │  (Physical = real HW, Fake = sim_mode)
          └─────┬───────────────┘
                │               │
         motion ok          motion error
                │               │
                ▼               ▼
             Idle            Error
```

`sim_mode` is a ROS parameter (`false` by default). When `true`, `Status/Overall` shows `"Fake"` during execution instead of `"Physical"`.

---

## 11. Last Will Testament (NDEATH)

Set on the MQTT client before `connect()` is called. If the bridge process crashes or loses connectivity without a clean disconnect, the broker publishes this automatically.

| Property | Value |
|---|---|
| Topic | `spBv1.0/DMAT/NDEATH/agx_arm_bridge` |
| Payload | Tahu `getNodeDeathPayload()` (contains `bdSeq`) |
| QoS | 1 |
| Retain | No |

---

## 12. ROS Interface

| ROS Topic | Type | Direction | Description |
|---|---|---|---|
| `feedback/joint_states` | `sensor_msgs/JointState` | Subscribe | Joint feedback from real hardware |
| `control/joint_states` | `sensor_msgs/JointState` | Subscribe | Joint feedback from simulation |

Both topics write to the same cache — whichever publishes last wins. This allows the bridge to work in both sim and real hardware without reconfiguration.
