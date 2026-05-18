# Packaging Line — SCADA / MES / Visualization Integration Specification

Authoritative contract for the **Sparkplug B** data model exposed by every
station on the packaging line. Consumed by:

- **Ignition SCADA** (Cirrus Link MQTT Engine module) — HMI, historian,
  alarm journal, broker of writes back to PLC / nodes.
- **Sepasoft MES** (running inside Ignition) — OEE, Track & Trace,
  SPC, Batch / Recipe.
- **Unity 3D digital twin** — passive subscriber for real-time
  visualisation of cell state.

Document version: **2.0** (full rewrite, replaces v1 legacy schema).
Last revised: 2026-05-11.

---

## 1. Overview

### 1.1 The line

Six stations plus a conveyor, producing filled cups of pellets:

```
            ┌──────────────────────────── conveyor (continuous) ────────────────────────────┐
            │                                                                                │
            ▼                                                                                │
    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
    │  Dispense    │ ─► │     Fill     │ ─► │    Weigh     │ ─► │    Vision    │ ─► │     Sort     │ ─► │    Robot     │ ──┐
    │ (cup feeder) │    │  (extruder)  │    │   (scale)    │    │  (camera)    │    │  (diverter)  │    │  (AGX arm)   │   │
    └──────────────┘    └──────────────┘    └──────────────┘    └──────────────┘    └──────────────┘    └──────────────┘   │
            ▲                                                                                                                │
            │                                                                                                                │
            └──────────────── empty/clean cup return ────────────────────────────────────────────────────────────────────────┘
```

The Robot closes the loop: it picks rejected/finished cups out of the
Sort bins, empties their content, and returns the empty cup to the
Dispense queue.

### 1.2 Control responsibility

| Subsystem        | Controller                | Sparkplug role                 |
|------------------|---------------------------|--------------------------------|
| Conveyor         | PLC (IEC 61131-3)         | Device under PLC EoN node      |
| Dispense         | PLC                       | Device under PLC EoN node      |
| Fill             | PLC                       | Device under PLC EoN node      |
| Weigh            | PLC                       | Device under PLC EoN node      |
| Sort             | PLC                       | Device under PLC EoN node      |
| Vision           | Vision PC                 | **Own EoN node**, 1 device     |
| Robot (AGX Arm)  | ROS 2 bridge (this repo)  | **Own EoN node**, 1 device     |
| Safety (E-Stop, guards, light curtain) | SIL-rated safety PLC | Read-only into the model (§14) |

### 1.3 What this document is not

- Not a wiring diagram. Hardware I/O is the PLC vendor's responsibility.
- Not a safety document. ISA-84 / IEC 61511 safety functions are in
  hardware and are mirrored read-only into Sparkplug (§14).
- Not an HMI design spec. ISA-101 graphic conventions are a separate
  Ignition project document.

---

## 2. System architecture

### 2.1 Network zones (IEC 62443 §3-2)

```
   ┌─────────────────────────────────────────────────────────────────┐
   │  Enterprise Zone (IT)                                           │
   │     ▸ ERP                                                       │
   └────────────────────────────┬────────────────────────────────────┘
                                │   firewall, REST/HTTPS only
   ┌────────────────────────────┴────────────────────────────────────┐
   │  DMZ                                                            │
   │     ▸ Ignition Gateway (Primary Host) + Sepasoft MES modules    │
   │     ▸ MQTT broker (HiveMQ / Cirrus Link Chariot)                │
   │     ▸ Historian (Ignition Tag Historian)                        │
   └────────────────────────────┬────────────────────────────────────┘
                                │   TLS 1.2+ with per-node client certs
   ┌────────────────────────────┴────────────────────────────────────┐
   │  Cell Zone (OT)                                                 │
   │                                                                 │
   │   EoN: PLC                EoN: Vision PC      EoN: Robot Bridge │
   │   ──────────────────      ─────────────────   ───────────────── │
   │   Devices:                Device:             Device:           │
   │     • Conveyor              • Vision            • Robot         │
   │     • Dispense                                                  │
   │     • Fill                                                      │
   │     • Weigh                                                     │
   │     • Sort                                                      │
   │                                                                 │
   │   Safety PLC (SIL-rated, NOT a Sparkplug node — DI only)        │
   └─────────────────────────────────────────────────────────────────┘

   Unity client connects from anywhere — read-only Sparkplug subscriber,
   no write authority, broker ACL enforced.
```

### 2.2 Sparkplug node inventory

| EoN node id           | Host                | Devices                                            |
|-----------------------|---------------------|----------------------------------------------------|
| `PLC`                 | Line PLC            | `Conveyor`, `Dispense`, `Fill`, `Weigh`, `Sort`    |
| `Vision`              | Vision PC           | `Vision`                                           |
| `Robot`               | Robot Bridge (ROS 2)| `Robot`                                            |

### 2.3 Sparkplug identifiers

All three EoN nodes share the same group and cell:

| Field             | Value                  | ISA-88 level         |
|-------------------|------------------------|----------------------|
| `group_id`        | `<Site>_<Area>`        | Site + Area          |
| `edge_node_id`    | `Packaging`            | Process Cell (the EoN node id is shared by *all three nodes* with a suffix — see below) |
| `device_id`       | one of: `Conveyor` / `Dispense` / `Fill` / `Weigh` / `Sort` / `Vision` / `Robot` | Unit |

Sparkplug requires `edge_node_id` to be unique per **host process**.
Use a suffix to differentiate the three EoN nodes while keeping the
cell association obvious:

| Host                | Concrete `edge_node_id` |
|---------------------|-------------------------|
| Line PLC            | `Packaging_PLC`         |
| Vision PC           | `Packaging_Vision`      |
| Robot Bridge        | `Packaging_Robot`       |

`primary_host_id` is `IgnitionGW` on all nodes — they all monitor the
same Primary Host.

---

## 3. Standards & conformance

| Standard                          | Role in this spec                                    | Conformance |
|-----------------------------------|------------------------------------------------------|-------------|
| **Sparkplug B v3.0** (Eclipse)    | Wire protocol, topic namespace, payload, lifecycle    | Full        |
| **ISA-88 / IEC 61512**            | Physical equipment hierarchy + procedural model       | Full (§4, §7) |
| **ISA-95 / IEC 62264**            | Level 2 ↔ 3 standard messages, equipment model        | Full (§7.4) |
| **ISA-TR88.00.02 / OMAC PackML**  | State machine + **PackTags v3.02** naming dictionary  | Full (§7.2, App. A) |
| **ISA-18.2 / IEC 62682**          | Alarm lifecycle (state, priority, ack, shelve, RTN)   | Full (§7.7, §13)    |
| **NAMUR NE107**                   | Diagnostic severity scale                             | Used as priority mapping (§7.7) |
| **IEC 62443**                     | Cybersecurity zones & conduits                        | Topology (§2.1), transport (§4.2) |
| **IEC 61131-3**                   | PLC programming languages                             | UDTs mirror IEC 61131-3 structured types (§7) |
| **ISA-84 / IEC 61511**            | Functional safety                                     | Boundary only — hardware (§14)  |
| **VDMA 75201 / UA-Rob**           | OPC UA Robotics companion spec                        | Folder names align for future OPC UA bridge |

---

## 4. Sparkplug B transport

### 4.1 Topic map

Namespace fixed at `spBv1.0`. `<G>` = `group_id`, `<N>` = `edge_node_id`,
`<D>` = `device_id`, `<H>` = `primary_host_id`.

| Direction      | Topic                                  | QoS | Retained | Purpose                                |
|----------------|----------------------------------------|-----|----------|----------------------------------------|
| Node → broker  | `spBv1.0/<G>/NBIRTH/<N>`               | 1   | no       | Node birth                             |
| Node → broker  | `spBv1.0/<G>/NDEATH/<N>`               | 1   | no       | Registered as MQTT LWT before connect  |
| Node → broker  | `spBv1.0/<G>/DBIRTH/<N>/<D>`           | 1   | no       | Device birth                           |
| Node → broker  | `spBv1.0/<G>/DDEATH/<N>/<D>`           | 1   | no       | Device offline                         |
| Node → broker  | `spBv1.0/<G>/DDATA/<N>/<D>`            | 0   | no       | RBE telemetry                          |
| Broker → node  | `spBv1.0/<G>/NCMD/<N>`                 | 1   | no       | Node control writes                    |
| Broker → node  | `spBv1.0/<G>/DCMD/<N>/<D>`             | 1   | no       | Device control writes                  |
| Primary → all  | `spBv1.0/STATE/<H>`                    | 1   | yes      | Primary Host online indicator          |

### 4.2 Connection lifecycle (Sparkplug 3.0 §5)

Every EoN node must:

1. Compute `bdSeq` (0..255, increments per node lifetime, persisted).
2. Register NDEATH as MQTT LWT **before** `connect`. Payload carries
   `bdSeq`.
3. Connect over TLS 1.2+, with per-node client certificate and broker
   CA pinned.
4. On connect-ack:
   - Subscribe `NCMD/<N>`, `DCMD/<N>/+`, `STATE/<H>`.
   - Publish NBIRTH (`seq=0`, same `bdSeq`).
   - Publish DBIRTH for every device, sequence continues incrementing.
5. Reset RBE caches on every reconnect — DBIRTH re-establishes ground
   truth for SCADA.
6. On `STATE/<H> = {online: false}` → enter Safe State (§7.6).

### 4.3 Node Control metrics (NCMD)

Every EoN node publishes the following in NBIRTH and honours them on
NCMD:

| Metric                          | Type    | Behaviour                                          |
|---------------------------------|---------|----------------------------------------------------|
| `Node Control/Rebirth`          | Boolean | Republish NBIRTH + DBIRTH for all devices          |
| `Node Control/Reboot`           | Boolean | Restart the node process (audit-logged)            |
| `Node Control/Next Server`      | Boolean | Disconnect and connect to the next broker in the HA list |
| `Node Control/Scan Rate`        | Int64   | Override telemetry timer period (ms)               |

### 4.4 Metric PropertySet (Sparkplug 3.0 §6.4.6)

Every numeric metric carries:

| Property        | Type    | Example                                         |
|-----------------|---------|-------------------------------------------------|
| `engUnit`       | String  | `deg`, `m`, `kg`, `g`, `°C`, `rpm`, `m/s`, `%`  |
| `engLow`        | Float   | Lower bound of expected operating range         |
| `engHigh`       | Float   | Upper bound                                     |
| `documentation` | String  | One-line human description                      |
| `displayName`   | String  | HMI-friendly label distinct from tag path       |
| `quality`       | Int32   | 0=Bad, 192=Good, 64=Uncertain                   |

Discrete metrics (`StatusCurrentState`, `Cmd/CntrlCmd`, fault codes)
additionally carry:

| Property        | Type    | Example                                         |
|-----------------|---------|-------------------------------------------------|
| `lookup`        | String  | JSON `{"1":"Stopped","2":"Starting",...}`       |

Metric flags used per Sparkplug 3.0:

- `is_historical=true` on backfilled DDATA after reconnect.
- `is_transient=true` on DCMD echoes (informational, not historised).
- `is_null=true` when "no current value" (e.g. `Material/PartId` when
  no part is in the cell) — never publish a sentinel string.

### 4.5 RBE & heartbeat

| Telemetry timer rate         | 10 Hz                                         |
|------------------------------|-----------------------------------------------|
| Full republish period        | 5 s (all metrics, regardless of deadband)     |
| Heartbeat metric             | `Status/Heartbeat` toggled at 2 Hz            |
| Stale-on-no-heartbeat        | SCADA marks node BAD after 3 s no toggle      |

Per-metric deadbands are declared in `engLow`/`engHigh` ratios per
device — typical values:

| Family                              | Deadband               |
|-------------------------------------|------------------------|
| Joint angle, EE rotation            | 0.1° (= 0.1% of 100°)  |
| EE translation                      | 0.5 mm                 |
| Conveyor / motor speed              | 1% of range            |
| Mass (Weigh)                        | 0.05 g                 |
| Gripper opening                     | 1% of range            |
| Counters                            | every change           |
| Discrete state / mode               | every change           |

---

## 5. Naming conventions

These rules apply uniformly across every node, every device, every
folder. The conventions are designed so any tag can be derived from
its purpose with no guessing.

### 5.1 Hierarchy

```
spBv1.0 / <Site>_<Area> / <NodeId> / <DeviceId> / <EquipmentModule> / <Role> / <Instance> / <Attribute>
```

Concrete example for joint 3 angle on the Robot:

```
spBv1.0/PlantA_Line1/Packaging_Robot/Robot/Motion/Joint/J3/Actual/Position
```

Each `/`-separated segment is one tier of meaning. Tags do not skip
tiers; if a tier is irrelevant for a metric it is omitted (e.g. status
flags don't carry `<Role>`).

### 5.2 Naming rules

1. **PascalCase** for every segment. No underscores in a segment.
   `JobComplete` not `Job_Complete`; `StationReady` not `Station_Ready`.
   Exception: id prefixes that ISA-88 sites use are allowed
   (`PlantA_Line1` as the group id).
2. **Singular containers** when an instance code follows.
   `Joint/J1` not `Joints/J1`. `Bin/A` not `Bins/A`. Plurals only on
   leaf collections with no per-instance addressing (`Counters/`).
3. **Role suffixes** — every paired value uses one of:
   - `…/Actual` — measured / current
   - `…/Target` — commanded by recipe or step
   - `…/Setpoint` — operator/recipe parameter
   - `…/Last` — most recently completed value (snapshot)
4. **Writability = folder**. Anything under `Cmd/` is DCMD writable;
   `Node Control/` is NCMD writable; everything else is read-only.
5. **Lookup-coded discretes** are always `Int32` with a `lookup`
   PropertySet — never `String` for state codes.
6. **Timestamps** are `Int64` epoch milliseconds with name ending in
   `…Ms` (`OnsetMs`, `LastTimestampMs`).
7. **Units** appear nowhere in the tag name — they're in the `engUnit`
   property. `Motion/Joint/J1/Actual/Position`, not `…/Position_deg`.

### 5.3 Reserved top-level folders

Every device has the same nine universal folders. Equipment-specific
folders are added as additional siblings.

| Folder        | Purpose                                                | Mandatory |
|---------------|--------------------------------------------------------|-----------|
| `Admin/`      | Static identity, versions, equipment class             | yes       |
| `Status/`     | Modal state (PackML, mode, blocked/starved, heartbeat) | yes       |
| `Handshake/`  | ISA-95 standard handshake (Job_Complete, Step, …)      | yes       |
| `Recipe/`     | ISA-S88 active recipe + phase + parameters             | yes       |
| `Material/`   | The part currently being handled                       | yes       |
| `Interlock/`  | Inter-unit and cell-wide interlocks                    | yes       |
| `Alarm/`      | ISA-18.2 alarm summary + active alarm list             | yes       |
| `Cmd/`        | DCMD writable commands                                 | yes       |
| `Counters/`   | OEE-friendly cycle counters                            | yes       |
| `Diag/`       | Non-operational diagnostics                            | yes       |

Equipment modules:

| Folder        | Stations using it           |
|---------------|-----------------------------|
| `Motor/`      | Conveyor, Fill              |
| `Cylinder/`   | Dispense                    |
| `Hopper/`     | Dispense, Fill              |
| `Extruder/`   | Fill                        |
| `Dosing/`     | Fill                        |
| `Shuttle/`    | Weigh                       |
| `Scale/`      | Weigh                       |
| `Quality/`    | Weigh, Vision               |
| `Camera/`     | Vision                      |
| `Lighting/`   | Vision                      |
| `Inspect/`    | Vision                      |
| `Model/`      | Vision                      |
| `Diverter/`   | Sort                        |
| `Bin/`        | Sort                        |
| `Decision/`   | Sort                        |
| `Motion/`     | Robot                       |
| `Gripper/`    | Robot                       |
| `Position/`   | Conveyor (zone sensors)     |
| `Belt/`       | Conveyor                    |

---

## 6. Universal tag tree

Tags present on every device. Type column is the Sparkplug metric
type. **W** in the RW column denotes DCMD writability; absence means
read-only.

### 6.1 `Admin/` — static identity

Published only in DBIRTH. Updated only on Rebirth.

| Tag                         | Type    | RW | Description                                       |
|-----------------------------|---------|----|---------------------------------------------------|
| `Admin/EquipmentId`         | String  |    | Per-device asset / serial                         |
| `Admin/EquipmentName`       | String  |    | Human label                                       |
| `Admin/EquipmentClass`      | String  |    | ISA-88 class: `Conveyor`, `Robot`, `Scale`, …     |
| `Admin/Site`                | String  |    | Mirrors group prefix                              |
| `Admin/Area`                | String  |    | Mirrors group suffix                              |
| `Admin/Cell`                | String  |    | Cell name (informational)                         |
| `Admin/Version/Firmware`    | String  |    |                                                   |
| `Admin/Version/Application` | String  |    | e.g. ROS package version, PLC project rev         |
| `Admin/Version/TagSpec`     | String  |    | This document version (`2.0`)                     |
| `Admin/PackMLSubset`        | String  |    | JSON array of supported PackML state codes        |
| `Admin/StopReason`          | String  |    | OMAC `AdminStopReason` — reason for last Stopped  |
| `Admin/MachDesignSpeed`     | Float   |    | OMAC `AdminMachDesignSpeed` — nameplate rate      |

### 6.2 `Status/` — OMAC PackTags v3.02 modal state

| Tag                              | OMAC PackTag                  | Type    | RW | Notes                                       |
|----------------------------------|-------------------------------|---------|----|---------------------------------------------|
| `Status/State/Current`           | `StatusCurrentState`          | Int32   |    | PackML state code 1..17 (App. A)            |
| `Status/State/CurrentName`       | (derived)                     | String  |    | Mirror, for HMI convenience                 |
| `Status/State/Requested`         | `StatusStateRequested`        | Int32   |    | Last requested via `Cmd/CntrlCmd`           |
| `Status/UnitMode/Current`        | `StatusUnitModeCurrent`       | Int32   |    | 1=Production, 2=Maintenance, 3=Manual, 4=Cleaning |
| `Status/UnitMode/CurrentName`    | (derived)                     | String  |    |                                             |
| `Status/Remote`                  | `StatusRemoteCmd`             | Boolean |    | true = command from SCADA accepted          |
| `Status/MachSpeed/Setpoint`      | `StatusMachSpeed`             | Float   |    | Requested production rate                   |
| `Status/MachSpeed/Current`       | `StatusCurMachSpeed`          | Float   |    | Measured production rate                    |
| `Status/Blocked`                 | `StatusBlocked`               | Boolean |    | Downstream cannot accept                    |
| `Status/Starved`                 | `StatusStarved`               | Boolean |    | Upstream not providing                      |
| `Status/Heartbeat`               | (Sparkplug convention)        | Boolean |    | 2 Hz toggle                                 |

### 6.3 `Handshake/` — ISA-95 station handshake

| Tag                              | Type    | RW | Notes                                                 |
|----------------------------------|---------|----|-------------------------------------------------------|
| `Handshake/StationReady`         | Boolean |    | False on E-Stop, fault, or Primary Host offline       |
| `Handshake/ExecuteCmd`           | Boolean |    | Mirrors `Cmd/CntrlCmd=Start` (informational)          |
| `Handshake/Processing`           | Boolean |    | True between accept and complete                      |
| `Handshake/JobComplete`          | Boolean |    | Latched on cycle complete; cleared by `Cmd/JobCompleteAck=false` |
| `Handshake/CurrentStep`          | Int32   |    | Sub-step within the active cycle (App. B)             |

### 6.4 `Recipe/` — ISA-S88 procedural

| Tag                                          | Type    | RW | Notes                                       |
|----------------------------------------------|---------|----|---------------------------------------------|
| `Recipe/Active/Id`                           | String  |    | Recipe UUID                                 |
| `Recipe/Active/Name`                         | String  |    |                                             |
| `Recipe/Active/Version`                      | String  |    |                                             |
| `Recipe/Active/LoadedAtMs`                   | Int64   |    | epoch ms when recipe became active          |
| `Recipe/Phase/Current`                       | String  |    | Phase name                                  |
| `Recipe/Phase/CurrentState`                  | Int32   |    | PhaseState 1..13 (App. C)                   |
| `Recipe/Phase/StepIndex`                     | Int32   |    |                                             |
| `Recipe/Phase/StepCount`                     | Int32   |    |                                             |
| `Recipe/Parameters/<name>/Value`             | varies  |    | One sub-tree per parameter, station-specific |
| `Recipe/Parameters/<name>/Unit`              | String  |    |                                             |

### 6.5 `Material/` — part-in-hand

| Tag                                | Type   | RW | Notes                                       |
|------------------------------------|--------|----|---------------------------------------------|
| `Material/PartId/Current`          | String |    | `is_null=true` when no part is in the station |
| `Material/PartId/Last`             | String |    | Previous part (for trace correlation)        |
| `Material/Class`                   | String |    | e.g. cup variant, pellet grade               |
| `Material/Source/Station`          | String |    | Producer device_id                           |
| `Material/Source/Lane`             | Int32  |    | Slot / lane on producer                      |
| `Material/Destination/Station`     | String |    | Next station                                 |
| `Material/Destination/Bin`         | String |    | e.g. "GoodA", "RejectB"                      |
| `Material/EnteredAtMs`             | Int64  |    | When this part entered the station           |

### 6.6 `Interlock/` — cell-wide and inter-unit

| Tag                                          | Type    | RW | Notes                                       |
|----------------------------------------------|---------|----|---------------------------------------------|
| `Interlock/EStop`                            | Boolean |    | Cell-wide E-Stop (mirror from safety PLC)   |
| `Interlock/GuardClosed`                      | Boolean |    | All guards closed                            |
| `Interlock/LightCurtainClear`                | Boolean |    |                                             |
| `Interlock/AirPressureOk`                    | Boolean |    | Pneumatic supply nominal                    |
| `Interlock/PowerOk`                          | Boolean |    | 24 V control bus nominal                    |
| `Interlock/Upstream/<peer>/MaterialAvailable`| Boolean |    | Per upstream peer device                    |
| `Interlock/Upstream/<peer>/Permit`           | Boolean |    | Peer permits hand-off                       |
| `Interlock/Downstream/<peer>/Ready`          | Boolean |    | Per downstream peer device                  |
| `Interlock/Downstream/<peer>/Acknowledged`   | Boolean |    | Downstream took the part                    |

### 6.7 `Alarm/` — ISA-18.2 lifecycle

| Tag                                          | Type    | RW | Notes                                                 |
|----------------------------------------------|---------|----|-------------------------------------------------------|
| `Alarm/Summary/ActiveCount`                  | Int32   |    |                                                       |
| `Alarm/Summary/UnacknowledgedCount`          | Int32   |    |                                                       |
| `Alarm/Summary/HighestPriority`              | Int32   |    | 1=Critical..4=Diagnostic                              |
| `Alarm/Summary/HighestSeverityName`          | String  |    | NAMUR NE107 label                                     |
| `Alarm/Active/<code>/State`                  | Int32   |    | 1=Normal,2=Unack,3=Ack,4=RTN,5=Shelved,6=Suppressed   |
| `Alarm/Active/<code>/StateName`              | String  |    |                                                       |
| `Alarm/Active/<code>/Priority`               | Int32   |    | 1..4                                                  |
| `Alarm/Active/<code>/Message`                | String  |    | Operator-actionable one-liner                         |
| `Alarm/Active/<code>/OnsetMs`                | Int64   |    |                                                       |
| `Alarm/Active/<code>/AckMs`                  | Int64   |    |                                                       |
| `Alarm/Active/<code>/AckUser`                | String  |    | From MES auth context                                 |
| `Alarm/History/Last10`                       | String  |    | Rolling JSON window                                   |

Active alarms are sparse — a code only appears under `Alarm/Active/`
when its state is non-Normal. SCADA discovers them via DBIRTH.

### 6.8 `Cmd/` — DCMD writable

| Tag                                          | Type    | RW  | Notes                                                 |
|----------------------------------------------|---------|-----|-------------------------------------------------------|
| `Cmd/CntrlCmd`                               | Int32   | W   | OMAC `CommandCntrlCmd`. Codes in App. D               |
| `Cmd/UnitMode/Requested`                     | Int32   | W   | 1..4 (Production/Maint/Manual/Cleaning)               |
| `Cmd/MachSpeed/Requested`                    | Float   | W   |                                                       |
| `Cmd/MaterialInterlock/Blocked`              | Boolean | W   | Force-blocked from SCADA                              |
| `Cmd/MaterialInterlock/Starved`              | Boolean | W   | Force-starved from SCADA                              |
| `Cmd/Recipe/Load`                            | String  | W   | Load recipe by Id                                     |
| `Cmd/Recipe/Phase/Request`                   | String  | W   | Force a phase transition (Maintenance mode only)      |
| `Cmd/Alarm/Acknowledge`                      | Int32   | W   | Alarm code; 0 = ack all                               |
| `Cmd/Alarm/Shelve`                           | Int32   | W   | Alarm code; auto-unshelves per policy                 |
| `Cmd/Alarm/Reset`                            | Boolean | W   | Reset latched alarm summary                           |

Every successful DCMD write is echoed back as DDATA with
`is_transient=true` so the Ignition tag browser reflects the last
written value without persisting a history point.

### 6.9 `Counters/` — OEE primitives

| Tag                                          | Type    | RW | Notes                                       |
|----------------------------------------------|---------|----|---------------------------------------------|
| `Counters/Cycles/Total`                      | Int64   |    | OMAC `AdminProdProcessedCount`              |
| `Counters/Cycles/Good`                       | Int64   |    | OMAC `AdminProdConsumedCount` (passed)      |
| `Counters/Cycles/Faulted`                    | Int64   |    | OMAC `AdminProdDefectiveCount`              |
| `Counters/Cycles/AverageMs`                  | Float   |    | Moving average cycle time                   |
| `Counters/Runtime/Seconds`                   | Int64   |    | OMAC `AdminAccTimeCount` (in Execute)       |
| `Counters/Downtime/Seconds`                  | Int64   |    | Total time in Stopped/Aborted               |
| `Counters/SinceReset/Cycles`                 | Int64   |    | Resets via `Cmd/Alarm/Reset` or shift start |

### 6.10 `Diag/`

| Tag                                          | Type    | RW | Notes                                       |
|----------------------------------------------|---------|----|---------------------------------------------|
| `Diag/CPULoad`                               | Float   |    | %                                            |
| `Diag/MemoryUsed`                            | Float   |    | %                                            |
| `Diag/Network/RTTToBroker`                   | Float   |    | ms                                           |
| `Diag/Network/LastReconnectMs`               | Int64   |    |                                             |
| `Diag/Network/ReconnectCount`                | Int32   |    | Since process start                          |
| `Diag/Temperature/Controller`                | Float   |    | °C                                           |
| `Diag/Lifetime/Hours`                        | Float   |    | h                                            |

---

## 7. Per-station tag trees

For each station: **universal folders are assumed present** (§6) and
not re-listed. Only the equipment-specific tags are tabulated.

### 7.1 Conveyor — `Packaging_PLC / Conveyor`

ISA-88 EquipmentClass: `Conveyor`.

Physical: continuous belt with N photoelectric position sensors,
geared induction motor on a VFD.

#### Equipment tags

| Tag                                          | Type    | Unit | RW | Description                          |
|----------------------------------------------|---------|------|----|--------------------------------------|
| `Motor/Speed/Setpoint`                       | Float   | m/s  | W  |                                      |
| `Motor/Speed/Actual`                         | Float   | m/s  |    |                                      |
| `Motor/Running`                              | Boolean |      |    |                                      |
| `Motor/Direction`                            | Int32   |      |    | 1=fwd, -1=rev                        |
| `Motor/Load`                                 | Float   | %    |    | VFD reported load                    |
| `Motor/Current`                              | Float   | A    |    |                                      |
| `Belt/TensionOk`                             | Boolean |      |    |                                      |
| `Belt/SlipDetected`                          | Boolean |      |    |                                      |
| `Belt/TotalDistance`                         | Float   | m    |    | Lifetime travel                      |
| `Position/Sensor/S1/Triggered` ..`S6`        | Boolean |      |    | One per zone — see naming below      |
| `Position/Sensor/S1/Name` ..`S6`             | String  |      |    | `AtDispense`, `AtFill`, `AtWeigh`, `AtVision`, `AtSort`, `AtRobot` |
| `Position/Sensor/S1/LastEdgeMs` ..           | Int64   |      |    | epoch ms of last rising edge         |
| `Position/CupTracking/<id>/Zone`             | String  |      |    | Per-cup tracker (optional)           |
| `Position/CupTracking/<id>/EnteredZoneMs`    | Int64   |      |    |                                      |

#### Recipe parameters

| Parameter                  | Unit  | Default | Notes                          |
|----------------------------|-------|---------|--------------------------------|
| `LineSpeed`                | m/s   | 0.10    | Default belt speed             |
| `EmergencyStopRamp`        | s     | 0.5     | Deceleration time              |

### 7.2 Dispense — `Packaging_PLC / Dispense`

ISA-88 EquipmentClass: `CupFeeder`.

Physical: pneumatic cylinder strokes once per cup; hopper gravity-fed
from above.

#### Equipment tags

| Tag                                          | Type    | Unit | RW | Description                            |
|----------------------------------------------|---------|------|----|----------------------------------------|
| `Cylinder/Extended`                          | Boolean |      |    | Reed switch on extended position       |
| `Cylinder/Retracted`                         | Boolean |      |    | Reed switch on retracted position      |
| `Cylinder/Position`                          | String  |      |    | `Extended` \| `Retracted` \| `Moving`  |
| `Cylinder/StrokeTime/Last`                   | Float   | ms   |    |                                        |
| `Cylinder/StrokeTime/Average`                | Float   | ms   |    | Rolling average                        |
| `Cylinder/StrokeCount/Total`                 | Int64   |      |    | Lifetime                               |
| `Cylinder/AirPressure`                       | Float   | bar  |    |                                        |
| `Hopper/Level/Percent`                       | Float   | %    |    |                                        |
| `Hopper/Level/Low`                           | Boolean |      |    | Triggers alarm 2002                    |
| `Hopper/Level/Empty`                         | Boolean |      |    | Triggers alarm 2001                    |
| `Hopper/Cups/Remaining`                      | Int32   |      |    |                                        |
| `Hopper/Cups/RefillThreshold`                | Int32   |      |    |                                        |

#### Recipe parameters

| Parameter                  | Unit  | Default | Notes                          |
|----------------------------|-------|---------|--------------------------------|
| `CupType`                  | —     | `50ml`  | Identifies cup variant         |
| `StrokeHoldTime`           | ms    | 200     | Dwell at extended              |
| `MaxStrokeTime`            | ms    | 1500    | Watchdog limit                 |

#### Cycle

`Cmd/CntrlCmd=Start` → Cylinder extends → cup released → cylinder
retracts → `Counters/Cycles/Good++` → `Status/State/Current=Complete`.

### 7.3 Fill — `Packaging_PLC / Fill`

ISA-88 EquipmentClass: `Extruder`.

Physical: hopper of pellets, screw extruder dosing into cup, gravimetric
feedback indirectly via Weigh.

#### Equipment tags

| Tag                                          | Type    | Unit | RW | Description                            |
|----------------------------------------------|---------|------|----|----------------------------------------|
| `Motor/Speed/Setpoint`                       | Float   | rpm  | W  |                                        |
| `Motor/Speed/Actual`                         | Float   | rpm  |    |                                        |
| `Motor/Running`                              | Boolean |      |    |                                        |
| `Motor/Current`                              | Float   | A    |    |                                        |
| `Motor/Temperature`                          | Float   | °C   |    |                                        |
| `Extruder/ScrewPosition`                     | Float   | rev  |    | Cumulative                             |
| `Hopper/Level/Percent`                       | Float   | %    |    |                                        |
| `Hopper/Level/Low`                           | Boolean |      |    | Alarm 3001                             |
| `Hopper/MassEstimate`                        | Float   | kg   |    | Inferred from level                    |
| `Dosing/Target/Mass`                         | Float   | g    |    | From `Recipe/Parameters/TargetMass`    |
| `Dosing/Dispensed/Mass`                      | Float   | g    |    | Live during cycle (open-loop estimate) |
| `Dosing/Dispensed/Time`                      | Float   | s    |    |                                        |
| `Dosing/Error/Mass`                          | Float   | g    |    | Computed once weigh reports back       |
| `Dosing/WithinTolerance`                     | Boolean |      |    |                                        |

#### Recipe parameters

| Parameter                  | Unit  | Default | Notes                                 |
|----------------------------|-------|---------|---------------------------------------|
| `TargetMass`               | g     | 25.0    |                                       |
| `PelletGrade`              | —     | `A`     |                                       |
| `ExtruderSpeed`            | rpm   | 120     | May be derived from TargetMass        |
| `Tolerance`                | g     | 0.5     | Pass band ± value                     |

### 7.4 Weigh — `Packaging_PLC / Weigh`

ISA-88 EquipmentClass: `Scale`.

Physical: stepper moves cup off-belt onto load cell, dwells, returns
to belt. Mass is the primary quality gate.

#### Equipment tags

| Tag                                          | Type    | Unit  | RW | Description                       |
|----------------------------------------------|---------|-------|----|-----------------------------------|
| `Shuttle/Position/Actual`                    | Int32   | steps |    |                                   |
| `Shuttle/Position/Target`                    | Int32   | steps |    |                                   |
| `Shuttle/AtScale`                            | Boolean |       |    | At weighing position              |
| `Shuttle/AtConveyor`                         | Boolean |       |    | At return position                |
| `Shuttle/Moving`                             | Boolean |       |    |                                   |
| `Shuttle/HomingComplete`                     | Boolean |       |    |                                   |
| `Scale/Mass/Current`                         | Float   | g     |    | Live load cell                    |
| `Scale/Mass/Last`                            | Float   | g     |    | Last completed reading            |
| `Scale/Mass/Tare`                            | Float   | g     |    |                                   |
| `Scale/Stable`                               | Boolean |       |    | Mass settled                      |
| `Scale/Calibration/LastTimestampMs`          | Int64   |       |    |                                   |
| `Scale/Calibration/Span`                     | Float   | g     |    |                                   |
| `Scale/Calibration/Drift`                    | Float   | g     |    |                                   |
| `Quality/Last/Result`                        | Int32   |       |    | -1=under, 0=pass, 1=over          |
| `Quality/Last/ResultName`                    | String  |       |    |                                   |
| `Quality/Last/Tolerance`                     | Float   | g     |    | From recipe                       |
| `Quality/Pass/Count`                         | Int64   |       |    |                                   |
| `Quality/Fail/Count`                         | Int64   |       |    |                                   |

#### Recipe parameters

| Parameter                  | Unit  | Default | Notes                                 |
|----------------------------|-------|---------|---------------------------------------|
| `SettleTime`               | s     | 1.0     | Dwell on load cell before reading     |
| `LowerLimit`               | g     | 24.5    |                                       |
| `UpperLimit`               | g     | 25.5    |                                       |
| `ShuttleSpeed`             | steps/s| 4000   |                                       |

### 7.5 Vision — `Packaging_Vision / Vision`

ISA-88 EquipmentClass: `VisionSystem`. Its own EoN node.

Physical: line-scan or area camera over a backlit fixture, ML
inference on a GPU, classifies pellet defects.

#### Equipment tags

| Tag                                          | Type    | Unit | RW | Description                              |
|----------------------------------------------|---------|------|----|------------------------------------------|
| `Camera/Online`                              | Boolean |      |    |                                          |
| `Camera/Exposure/Setpoint`                   | Int32   | μs   | W  |                                          |
| `Camera/Exposure/Actual`                     | Int32   | μs   |    |                                          |
| `Camera/Gain`                                | Float   |      |    |                                          |
| `Camera/Frame/Rate`                          | Float   | fps  |    |                                          |
| `Camera/Frame/LastTimestampMs`               | Int64   |      |    |                                          |
| `Lighting/Ring/On`                           | Boolean |      | W  |                                          |
| `Lighting/Ring/Intensity`                    | Float   | %    | W  |                                          |
| `Inspect/Trigger`                            | Boolean |      | W  | Edge-triggered one-shot inspection       |
| `Inspect/Result/Last/Classification`         | String  |      |    | `Pass` \| `Fail` \| `Defect_<kind>`     |
| `Inspect/Result/Last/ClassCode`              | Int32   |      |    | Lookup-coded                             |
| `Inspect/Result/Last/Confidence`             | Float   | 0..1 |    |                                          |
| `Inspect/Result/Last/DefectCount`            | Int32   |      |    |                                          |
| `Inspect/Result/Last/ImageRef`               | String  |      |    | URI to saved image                       |
| `Inspect/Result/Last/PartId`                 | String  |      |    | Cup this result belongs to               |
| `Inspect/Result/Last/TimestampMs`            | Int64   |      |    |                                          |
| `Model/Version`                              | String  |      |    |                                          |
| `Model/TrainedAtMs`                          | Int64   |      |    |                                          |
| `Model/ConfidenceThreshold`                  | Float   | 0..1 | W  |                                          |
| `Quality/Window/PassRate`                    | Float   | %    |    | Rolling N-cup pass rate                  |
| `Quality/Window/Throughput`                  | Float   | fpm  |    |                                          |

#### Recipe parameters

| Parameter                  | Unit  | Default | Notes                                 |
|----------------------------|-------|---------|---------------------------------------|
| `ConfidenceThreshold`      | 0..1  | 0.85    |                                       |
| `ModelVersion`             | —     | `v3.2`  | Active inference model                |
| `InspectTimeout`           | ms    | 500     | Per-cup hard deadline                 |

### 7.6 Sort — `Packaging_PLC / Sort`

ISA-88 EquipmentClass: `Diverter`.

Physical: pneumatic diverter into one of two bins. Decision authority
is **Sepasoft MES**, with a default-to-reject fallback on timeout.

#### Equipment tags

| Tag                                          | Type    | Unit | RW | Description                              |
|----------------------------------------------|---------|------|----|------------------------------------------|
| `Diverter/Position/Current`                  | String  |      |    | `BinA` \| `BinB` \| `Moving`             |
| `Diverter/Position/Setpoint`                 | String  |      |    | Last commanded                           |
| `Diverter/Moving`                            | Boolean |      |    |                                          |
| `Diverter/StrokeTime/Last`                   | Float   | ms   |    |                                          |
| `Bin/A/Name`                                 | String  |      |    | `Good`                                   |
| `Bin/A/Count`                                | Int64   |      |    |                                          |
| `Bin/A/Capacity`                             | Int32   |      |    |                                          |
| `Bin/A/Full`                                 | Boolean |      |    | Alarm 6001 at threshold                  |
| `Bin/A/Empty`                                | Boolean |      |    |                                          |
| `Bin/B/Name`                                 | String  |      |    | `Reject`                                 |
| `Bin/B/Count`                                | Int64   |      |    |                                          |
| `Bin/B/Capacity`                             | Int32   |      |    |                                          |
| `Bin/B/Full`                                 | Boolean |      |    |                                          |
| `Decision/Pending/PartId`                    | String  |      |    | Cup awaiting decision                    |
| `Decision/Pending/Bin`                       | String  |      | W  | MES writes here within DecisionTimeout   |
| `Decision/Pending/RequestedAtMs`             | Int64   |      |    | When the bridge asked MES                |
| `Decision/Last/PartId`                       | String  |      |    |                                          |
| `Decision/Last/Bin`                          | String  |      |    |                                          |
| `Decision/Last/Reason`                       | String  |      |    | e.g. `VisionFail`, `MassOOS`, `Default`  |

#### Recipe parameters

| Parameter                  | Unit  | Default | Notes                                 |
|----------------------------|-------|---------|---------------------------------------|
| `RejectBin`                | —     | `B`     | Fallback bin on timeout / fault       |
| `DecisionTimeout`          | ms    | 200     | MES must respond within this          |
| `BinFullThreshold`         | %     | 90      | Of capacity                           |

### 7.7 Robot — `Packaging_Robot / Robot`

ISA-88 EquipmentClass: `Robot`. Its own EoN node, ROS 2 bridge.

Physical: 6-DOF AGX arm with parallel gripper. Loop-closure role:
empties cups from Sort bins and returns them to Dispense.

#### Equipment tags

| Tag                                                  | Type    | Unit  | RW | Description                            |
|------------------------------------------------------|---------|-------|----|----------------------------------------|
| `Motion/Joint/J1/Actual/Position` ..`J6`             | Float   | deg   |    | engLow=-180, engHigh=180               |
| `Motion/Joint/J1/Actual/Velocity` ..`J6`             | Float   | deg/s |    |                                        |
| `Motion/Joint/J1/Actual/Torque`    ..`J6`            | Float   | Nm    |    | Where available                        |
| `Motion/Joint/J1/Target/Position`  ..`J6`            | Float   | deg   |    | Current waypoint target                |
| `Motion/EndEffector/Actual/X`                        | Float   | m     |    |                                        |
| `Motion/EndEffector/Actual/Y`                        | Float   | m     |    |                                        |
| `Motion/EndEffector/Actual/Z`                        | Float   | m     |    |                                        |
| `Motion/EndEffector/Actual/Roll`                     | Float   | deg   |    | Intrinsic XYZ                          |
| `Motion/EndEffector/Actual/Pitch`                    | Float   | deg   |    |                                        |
| `Motion/EndEffector/Actual/Yaw`                      | Float   | deg   |    |                                        |
| `Motion/EndEffector/Target/X` .. `Yaw`               | Float   |       |    | Same units as Actual                   |
| `Motion/Path/ProgressPercent`                        | Float   | %     |    | Path completion within current segment |
| `Gripper/Opening/Actual`                             | Float   | 0..1  |    | Normalised opening fraction            |
| `Gripper/Opening/Target`                             | Float   | 0..1  |    |                                        |
| `Gripper/Force/Setpoint`                             | Float   | N     |    |                                        |
| `Gripper/Force/Actual`                               | Float   | N     |    |                                        |
| `Gripper/PartPresent`                                | Boolean |       |    | Object detected between jaws           |
| `Material/Operation/Current`                         | String  |       |    | `Reset` \| `Pick` \| `Empty` \| `Place` \| `Idle` |
| `Material/Operation/StepIndex`                       | Int32   |       |    |                                        |
| `Material/Operation/TotalSteps`                      | Int32   |       |    |                                        |

#### Recipe parameters

| Parameter                  | Unit  | Default | Notes                                 |
|----------------------------|-------|---------|---------------------------------------|
| `Speed`                    | 0..1  | 0.5     | Velocity scaling factor               |
| `Acceleration`             | 0..1  | 0.5     |                                       |
| `GripperForce`             | N     | 20      |                                       |
| `PickApproachHeight`       | m     | 0.05    | Above target Z                        |
| `PlaceApproachHeight`      | m     | 0.05    |                                       |

#### Phase model

The robot's full procedure (one cycle from idle back to idle):

| Phase Name      | Index | Description                                  |
|-----------------|-------|----------------------------------------------|
| `Approach`      | 1     | Move to pick-approach pose over Sort bin     |
| `Descend`       | 2     | Move down to grasp height                    |
| `Grasp`         | 3     | Close gripper on cup                         |
| `Lift`          | 4     | Retreat vertically                           |
| `Transport`     | 5     | Move to empty station                        |
| `Empty`         | 6     | Tilt + drain cup into reject chute           |
| `ReturnTransport`| 7    | Move to Dispense feed lane                   |
| `Place`         | 8     | Lower + release cup                          |
| `Retract`       | 9     | Return to home pose                          |

Each phase populates `Recipe/Phase/Current` and `Recipe/Phase/CurrentState`.

---

## 8. Cross-station data flow

The Sparkplug topic tree alone doesn't tell SCADA which station feeds
which. The flow is **broker-and-correlator-by-MES**: stations do not
subscribe to each other; they publish; Sepasoft MES is the correlator
that joins by `Material/PartId` and writes back to the appropriate
`Cmd/` tags.

### 8.1 Producer / consumer matrix

| Step | Producer    | Tag                                           | Consumer(s)        | Trigger                         |
|------|-------------|-----------------------------------------------|--------------------|---------------------------------|
| 0    | Conveyor    | `Position/Sensor/S1/Triggered` (AtDispense)   | MES                | rising edge → start Dispense    |
| 1    | MES         | `Cmd/CntrlCmd=Start` on Dispense              | Dispense           |                                 |
| 2    | Dispense    | `Status/State/Current=Complete` + `Material/PartId/Current` | MES  | end of cycle                    |
| 3    | Conveyor    | `Position/Sensor/S2/Triggered` (AtFill)       | MES                |                                 |
| 4    | MES         | `Cmd/CntrlCmd=Start` on Fill + writes `Recipe/Parameters/TargetMass` | Fill |             |
| 5    | Fill        | `Dosing/Dispensed/Mass`, `Status/State/Current=Complete` | MES     |                                 |
| 6    | Conveyor    | `Position/Sensor/S3/Triggered` (AtWeigh)      | MES                |                                 |
| 7    | MES         | `Cmd/CntrlCmd=Start` on Weigh                 | Weigh              |                                 |
| 8    | Weigh       | `Scale/Mass/Last`, `Quality/Last/Result`      | MES                | shuttle back complete           |
| 9    | Conveyor    | `Position/Sensor/S4/Triggered` (AtVision)     | MES                |                                 |
| 10   | MES         | `Cmd/Inspect/Trigger=true` on Vision          | Vision             |                                 |
| 11   | Vision      | `Inspect/Result/Last/*`                       | MES                | inference complete              |
| 12   | Conveyor    | `Position/Sensor/S5/Triggered` (AtSort)       | MES                |                                 |
| 13   | Sort        | `Decision/Pending/PartId` request             | MES                | cup arrived                     |
| 14   | MES         | `Decision/Pending/Bin` (writes A or B)        | Sort               | within 200 ms                   |
| 15   | Sort        | `Bin/<bin>/Count` increment                   | MES                | divert complete                 |
| 16   | Sort        | `Bin/A/Full` or `Bin/B/Full`                  | MES                | level threshold                 |
| 17   | MES         | `Cmd/CntrlCmd=Start` on Robot + `Material/Source/Bin` | Robot       | bin-empty cycle requested       |
| 18   | Robot       | `Material/Operation/Current` updates          | MES, Unity         | per phase                       |
| 19   | Robot       | `Status/State/Current=Complete`               | MES                | cup returned to Dispense        |

### 8.2 Correlation key

`Material/PartId/Current` is the **single line-wide correlation key**.
The PLC mints it at Dispense (UUID v7 — sortable, time-prefixed). MES
joins all downstream events on this id and persists the full part
history in the Sepasoft Track & Trace lot record.

When no part is in a station, the producer publishes
`Material/PartId/Current` with `is_null=true` rather than an empty
string — this is observable on the broker without polling.

---

## 9. Ignition SCADA integration

### 9.1 Gateway configuration

| Item                          | Value                                          |
|-------------------------------|------------------------------------------------|
| Edition                       | Ignition 8.1 LTS (or 8.3)                      |
| MQTT module                   | Cirrus Link **MQTT Engine** (subscriber)       |
| Primary Host enabled          | yes — `primary_host_id=IgnitionGW`             |
| Broker                        | Cirrus Link Chariot or HiveMQ Enterprise       |
| Tag provider                  | `MQTT Engine / PlantA_Line1 / Packaging_*`     |

### 9.2 UDT definitions

Define **one UDT per ISA-88 EquipmentClass**, reused for every
instance. Members map 1:1 to the Sparkplug tag tree.

```
[UDT] CommonDevice
   ├── Admin/         (struct)
   ├── Status/        (struct, including OMAC PackTags)
   ├── Handshake/     (struct)
   ├── Recipe/        (struct)
   ├── Material/      (struct)
   ├── Interlock/     (struct)
   ├── Alarm/         (struct)
   ├── Cmd/           (struct, writable)
   ├── Counters/      (struct)
   └── Diag/          (struct)

[UDT] Conveyor   inherits CommonDevice + Motor/, Belt/, Position/
[UDT] Dispense   inherits CommonDevice + Cylinder/, Hopper/
[UDT] Fill       inherits CommonDevice + Motor/, Extruder/, Hopper/, Dosing/
[UDT] Weigh      inherits CommonDevice + Shuttle/, Scale/, Quality/
[UDT] Vision     inherits CommonDevice + Camera/, Lighting/, Inspect/, Model/, Quality/
[UDT] Sort       inherits CommonDevice + Diverter/, Bin/, Decision/
[UDT] Robot      inherits CommonDevice + Motion/, Gripper/
```

Instance binding example:

```
Tag path:    [Plant]Packaging/Robot
UDT type:    Robot
Source:      [MQTT Engine]PlantA_Line1/Packaging_Robot/Robot
```

### 9.3 Alarm pipeline

| Property in `Alarm/Active/<code>/`   | Ignition Alarm property         |
|--------------------------------------|---------------------------------|
| `Priority`                           | Priority                        |
| `State` (Unack/Ack/RTN/...)          | (driven by Ignition state model)|
| `Message`                            | Display Name                    |
| `OnsetMs`                            | Active Time                     |
| `AckMs`                              | Ack Time                        |
| `AckUser`                            | Ack User                        |

Acknowledge from Ignition writes back via DCMD:
`Cmd/Alarm/Acknowledge=<code>`. The bridge echoes the new state in
DDATA. The Ignition alarm journal records ack user and time.

### 9.4 Historian retention

| Tag family                    | Sample mode | Retention | Notes                       |
|-------------------------------|-------------|-----------|-----------------------------|
| `Status/State/Current`        | on-change   | 5 years   | Drives OEE                  |
| `Counters/*`                  | 1 min       | 5 years   |                             |
| `Motion/Joint/*/Actual/*`     | RBE         | 90 days   | High volume                 |
| `Motion/EndEffector/Actual/*` | RBE         | 90 days   |                             |
| `Scale/Mass/Last`             | on-change   | 5 years   | SPC source                  |
| `Dosing/Error/Mass`           | on-change   | 5 years   | SPC source                  |
| `Inspect/Result/Last/Confidence` | on-change | 5 years  | SPC source                  |
| `Alarm/*`                     | journal     | 7 years   | Compliance                  |

### 9.5 Gated trigger pattern (HMI)

```
on TriggerButtonClicked(device):
    if not device.Handshake.StationReady:                 abort "Not ready"
    if device.Handshake.JobComplete:                      abort "Acknowledge previous cycle first"
    if device.Alarm.Summary.UnacknowledgedCount > 0:      abort "Unacknowledged alarms"

    write device.Cmd.CntrlCmd = 2  # Start
    wait device.Handshake.Processing == true              timeout 5 s
    # PackML moves Idle → Starting → Execute autonomously
    wait device.Handshake.JobComplete == true             timeout 120 s
    write device.Cmd.CntrlCmd = 1  # Reset (Stopped → Idle if needed)
```

---

## 10. Sepasoft MES integration

The Sepasoft MES suite runs as modules inside Ignition. Modules
consume the same Sparkplug tag tree — the integration is mostly about
which fields each module needs and how MES writes back.

### 10.1 Module → tag mapping

| Sepasoft module           | Source tags                                                                                       | Writes back                                       |
|---------------------------|----------------------------------------------------------------------------------------------------|---------------------------------------------------|
| **OEE Downtime**          | `Status/State/Current`, `Status/Blocked`, `Status/Starved`, `Admin/StopReason`                     | none (operator codes reasons in OEE UI)           |
| **OEE Production**        | `Counters/Cycles/Good`, `Counters/Cycles/Faulted`, `Counters/Cycles/Total`, `Status/MachSpeed/Current` | none                                          |
| **Track & Trace**         | `Material/PartId/Current`, `Material/Source/*`, `Material/Destination/*`, `Recipe/Active/Id`       | reads lot/serial; writes assigned PartId at Dispense via `Cmd/Recipe/Load` |
| **SPC**                   | `Scale/Mass/Last`, `Dosing/Error/Mass`, `Inspect/Result/Last/Confidence`, `Quality/Last/Tolerance` | none (control limits live in Sepasoft project)    |
| **Batch Procedure**       | `Recipe/Active/*`, `Recipe/Phase/*`                                                                | `Cmd/Recipe/Load`, `Cmd/Recipe/Phase/Request`, `Cmd/CntrlCmd` per phase |
| **Recipe / Changeover**   | `Recipe/Parameters/*`                                                                              | `Recipe/Parameters/*` via DCMD (only in Idle / Maintenance modes) |
| **Production Scheduler**  | `Status/State/Current` of every device                                                             | `Cmd/CntrlCmd=Start` on the Conveyor when a job releases |

### 10.2 Track & Trace lot record

Per cup, the lot record assembled by Sepasoft contains:

| Sepasoft field                | From tag                                                                  |
|-------------------------------|---------------------------------------------------------------------------|
| Lot Id                        | `Dispense.Material.PartId.Current` at dispense complete                   |
| Recipe                        | `Dispense.Recipe.Active.Id`                                               |
| Dispense Time                 | `Dispense.Status.State.Current` transition timestamp to `Complete`        |
| Filled Mass (estimated)       | `Fill.Dosing.Dispensed.Mass` at Fill complete                             |
| Measured Mass                 | `Weigh.Scale.Mass.Last` at Weigh complete                                 |
| Mass Quality                  | `Weigh.Quality.Last.Result`                                               |
| Inspect Classification        | `Vision.Inspect.Result.Last.Classification`                               |
| Inspect Confidence            | `Vision.Inspect.Result.Last.Confidence`                                   |
| Inspect Image                 | `Vision.Inspect.Result.Last.ImageRef`                                     |
| Sort Decision                 | `Sort.Decision.Last.Bin` (+ `Reason`)                                     |
| Robot Operation               | `Robot.Material.Operation.Current` last value                             |
| Cycle End Time                | `Sort.Decision.Last` transition timestamp                                 |

### 10.3 SPC control points

| Variable                                     | Control limits source            | Out-of-control action |
|----------------------------------------------|----------------------------------|----------------------|
| `Weigh.Scale.Mass.Last`                      | `Recipe.Parameters.Tolerance`    | Alarm 3003 (Fill)    |
| `Fill.Dosing.Error.Mass`                     | derived from Tolerance           | SPC alert + email    |
| `Vision.Inspect.Result.Last.Confidence`      | `Model.ConfidenceThreshold`      | Alarm 5002 (Vision)  |
| `Conveyor.Motor.Load`                        | static (e.g. <80%)               | SPC alert            |

### 10.4 Batch Procedure execution

Sepasoft Batch Procedure plays the ISA-S88 recipe by:

1. Writing `Cmd/Recipe/Load=<id>` to every device.
2. Waiting for `Recipe/Active/Id == <id>` confirmation.
3. For each phase in the procedure, writing
   `Cmd/Recipe/Phase/Request=<name>` and waiting for
   `Recipe/Phase/CurrentState=3 (Complete)` before advancing.
4. On phase failure (state ≥ 11 Aborted), aborting the batch and
   issuing `Cmd/CntrlCmd=Abort` to all involved devices.

### 10.5 Write authority

Only Sepasoft and Ignition operators with the **Cell Supervisor** role
may write to `Cmd/*` tags. Broker ACL enforces this — Unity and other
visualisation clients are publish-denied for the entire group.

---

## 11. Runtime interactions — MES ↔ SCADA ↔ Device

§9 and §10 describe what each system does *individually*. This section
describes how they cooperate at runtime, and what handshake guarantees
that a cycle is genuinely **completed** versus merely *appears* to be.

### 11.1 Roles and write authority

Ignition Gateway hosts both the SCADA HMI and the Sepasoft MES modules,
but their write authority over device `Cmd/*` tags is distinct and
enforced by Ignition role-based permissions plus broker ACL.

| Layer       | Owner             | Authority over `Cmd/*`                                                                 | Trigger source         |
|-------------|-------------------|----------------------------------------------------------------------------------------|------------------------|
| **Device**  | PLC / Vision PC / Robot Bridge | None — devices accept writes, never write to their own `Cmd/*`               | Physical I/O, ROS topics |
| **SCADA**   | Ignition operator | `Cmd/CntrlCmd`, `Cmd/Alarm/*`, `Cmd/UnitMode/Requested` — manual, while in `Manual` or `Maintenance` modes | HMI button click  |
| **MES**     | Sepasoft Batch / Scheduler | `Cmd/CntrlCmd`, `Cmd/Recipe/*`, `Cmd/MachSpeed/Requested`, `Decision/Pending/Bin` — automatic in `Production` mode | Production order, sensor edge, decision request |
| **ERP** (out of band) | n/a     | None — talks to MES via REST                                                            | Order release           |

The device's `Status/Remote` tag is the **gate**: when `false`, the
device ignores every `Cmd/*` write and only responds to local HMI on
the station. SCADA/MES check `Status/Remote == true` before issuing
any command.

`Status/UnitMode/Current` further governs which commands are
permitted:

| UnitMode      | SCADA HMI can write              | MES can write                              |
|---------------|----------------------------------|--------------------------------------------|
| `Production`  | `Cmd/Alarm/*`, `Cmd/CntrlCmd=Stop`/`Hold`/`Abort` (safety actions) | All `Cmd/*` (drives the line) |
| `Manual`      | All `Cmd/*` (jog/test)           | Read-only — won't start production cycles  |
| `Maintenance` | All `Cmd/*` including `Cmd/Recipe/Phase/Request` | Read-only                |
| `Cleaning`    | `Cmd/CntrlCmd=Stop`/`Reset` only | Read-only                                  |

### 11.2 The fundamental handshake

Every interaction between an upstream actor (SCADA or MES) and a
device follows the same six-step pattern. The handshake combines
**ISA-95 standard handshake tags** (§6.3) with **PackML state
transitions** (App. A) and **OMAC `CntrlCmd`** writes (App. D).

The handshake is what differentiates a completed cycle from a
fire-and-forget write: the upstream actor knows the device finished
because it sees specific state transitions in a specific order.

```
            UPSTREAM (MES / SCADA)                              DEVICE
                    │                                              │
   1. Pre-flight    │ read:  Status/State/Current == 3 (Idle)      │
      check         │        Handshake/StationReady == true        │
                    │        Status/Remote == true                 │
                    │        Alarm/Summary/UnacknowledgedCount==0  │
                    │        Interlock/EStop == false              │
                    │        (all must pass; else: abort)          │
                    │                                              │
   2. Load recipe   │ write: Cmd/Recipe/Load = "<id>"              │
      (if changing) │ ────────────────────────────────────────────►│  (load YAML / params)
                    │ wait:  Recipe/Active/Id == "<id>"            │
                    │        (timeout 2 s; else: fail)             │
                    │                                              │
   3. Start         │ write: Cmd/CntrlCmd = 2  (Start)             │
                    │ ────────────────────────────────────────────►│  Status/State/Current: 3 → 2 (Starting)
                    │ wait:  Status/State/Current == 5 (Execute)   │  Status/State/Current: 2 → 5 (Execute)
                    │        Handshake/Processing == true          │  Handshake/Processing = true
                    │        Handshake/CurrentStep ≥ 2             │  Handshake/CurrentStep = 2 (Acknowledged)
                    │        (timeout 2 s; else: Abort & alarm)    │
                    │                                              │
   4. Monitor       │ observe (continuous):                        │  Recipe/Phase/Current advances
      progress      │   Recipe/Phase/Current                       │  Recipe/Phase/CurrentState = 2 (Running)
                    │   Recipe/Phase/CurrentState                  │  Handshake/CurrentStep = 3 (Executing)
                    │   Handshake/CurrentStep                      │  device-specific live tags update
                    │   Counters/Cycles/AverageMs (sanity check)   │
                    │                                              │
   5. Completion    │ wait:  Status/State/Current == 16 (Complete) │  Status/State/Current: 5 → 15 → 16
      latch         │        Handshake/JobComplete == true         │  Handshake/JobComplete = true (LATCHED)
                    │        Handshake/CurrentStep == 4            │  Handshake/CurrentStep = 4 (Complete)
                    │        (timeout = recipe-specific; else:     │
                    │         Abort & alarm)                       │
                    │ read:  cycle data — Material/PartId/Current, │
                    │        Quality/Last/*, etc.                  │
                    │ persist to Track & Trace                     │
                    │                                              │
   6. Reset         │ write: Cmd/CntrlCmd = 1  (Reset)             │
                    │ ────────────────────────────────────────────►│  Status/State/Current: 16 → 14 (Resetting)
                    │ wait:  Status/State/Current == 3 (Idle)      │  Handshake/JobComplete = false (clears)
                    │        Handshake/JobComplete == false        │  Status/State/Current: 14 → 3 (Idle)
                    │        (timeout 2 s; else: fail)             │
                    │                                              │
                    │  ready for next cycle                        │
```

**Why this is robust:**

- Every wait has an explicit timeout. No silent stalls.
- `Handshake/JobComplete` is **latched** — it cannot be missed by a
  slow consumer. Even if MES reads on a 1 s scan, the latch persists
  until step 6 clears it.
- `Status/State/Current` is the **authoritative** completion signal,
  not `Handshake/JobComplete` alone — a device that latches
  `JobComplete=true` but stays in `Execute` is faulty and the
  mismatch is detectable.
- `Counters/Cycles/Good` increments only on the successful
  `Execute → Completing → Complete` path. A consumer that diff'd
  `Good` before and after the cycle proves end-to-end success without
  any tag race.

### 11.3 Multi-step verification: "completed" means all three

A cycle is considered **truly complete** only when all three of the
following are observed in order on the same `Material/PartId`:

1. **State**: `Status/State/Current` transitions to `16 (Complete)`.
2. **Latch**: `Handshake/JobComplete` rises to `true`.
3. **Counter**: `Counters/Cycles/Good` increments by exactly 1
   (or `Counters/Cycles/Faulted` increments if the cycle failed
   gracefully — distinguish from `Aborted`).

Any subset is suspicious:

| Observed                                | Interpretation                                  | Action                |
|-----------------------------------------|-------------------------------------------------|-----------------------|
| State=Complete, JobComplete=true, Good++ | Real success                                    | Reset and advance     |
| State=Complete, JobComplete=true, no counter change | Stuck counter — device bug              | Alert engineering     |
| State=Aborted, JobComplete=false        | Real failure                                    | Diagnose, recover     |
| JobComplete=true but State=Execute      | Logic inversion in device firmware              | Alarm 9003 (Spec violation) |
| State=Complete but JobComplete=false    | Race — wait one more scan                       | Re-evaluate in 100 ms |

### 11.4 Operator-initiated cycle (SCADA / HMI)

Used in `Manual` and `Maintenance` modes, and as a recovery override
in `Production` mode. The operator clicks Start on a station's HMI
faceplate.

```
   OPERATOR              IGNITION HMI                  DEVICE
       │                       │                          │
       │  click Start          │                          │
       │ ────────────────────► │ (apply §11.2 pre-flight) │
       │                       │ ── DCMD Cmd/CntrlCmd=2 ─►│
       │                       │ ◄─ DDATA: State=5 ──────│
       │ Indicator: Running    │                          │
       │ ◄────────────────────│                          │
       │                       │                          │
       │                       │ ◄─ DDATA: State=16 ─────│
       │                       │     JobComplete=true     │
       │ Indicator: Complete   │                          │
       │ ◄────────────────────│                          │
       │  (auto-reset after    │                          │
       │   3 s, or manual)     │                          │
       │                       │ ── DCMD Cmd/CntrlCmd=1 ─►│
       │                       │ ◄─ DDATA: State=3 ──────│
       │ Indicator: Idle       │                          │
       │ ◄────────────────────│                          │
```

The HMI applies the same pre-flight checks as MES (§11.2 step 1).
Failures show as a banner above the Start button rather than firing
a DCMD into a non-ready device.

### 11.5 MES-orchestrated automatic cycle (the production case)

In `Production` mode, MES drives the entire line autonomously per
production order. The handshake compounds: MES tracks one
`PartId` through six stations, applying §11.2 to each station in
turn.

```
   ERP             SEPASOFT MES                  IGNITION SCADA            DEVICES
    │                  │                              │                        │
    │ Order release    │                              │                        │
    │ ───────────────► │ assign PartIds, load recipe  │                        │
    │                  │ ── Cmd/Recipe/Load to all ──────────────────────────► │
    │                  │ wait Recipe/Active/Id confirmed                       │
    │                  │ ── Cmd/CntrlCmd=Start → Conveyor ───────────────────► │
    │                  │                              │ belt running ───────► │
    │                  │                              │                        │
    │                  │ ┌── per cup ───────────────────────────────────────┐ │
    │                  │ │                                                  │ │
    │                  │ │ on Conveyor.Position/Sensor/S1/Triggered:        │ │
    │                  │ │   mint PartId; write Material/PartId on Dispense │ │
    │                  │ │   apply §11.2 handshake on Dispense              │ │
    │                  │ │                                                  │ │
    │                  │ │ on Conveyor.Position/Sensor/S2/Triggered:        │ │
    │                  │ │   apply §11.2 handshake on Fill                  │ │
    │                  │ │   harvest Dosing/Dispensed/Mass                  │ │
    │                  │ │                                                  │ │
    │                  │ │ on Conveyor.Position/Sensor/S3/Triggered:        │ │
    │                  │ │   apply §11.2 handshake on Weigh                 │ │
    │                  │ │   harvest Scale/Mass/Last, Quality/Last/Result   │ │
    │                  │ │                                                  │ │
    │                  │ │ on Conveyor.Position/Sensor/S4/Triggered:        │ │
    │                  │ │   write Cmd/Inspect/Trigger to Vision            │ │
    │                  │ │   harvest Inspect/Result/Last/*                  │ │
    │                  │ │                                                  │ │
    │                  │ │ on Conveyor.Position/Sensor/S5/Triggered:        │ │
    │                  │ │   wait Sort.Decision/Pending/PartId == thisPart  │ │
    │                  │ │   compute decision from prior data               │ │
    │                  │ │   write Sort.Decision/Pending/Bin                │ │
    │                  │ │   wait Sort.Decision/Last/Bin == decision        │ │
    │                  │ │                                                  │ │
    │                  │ │ append all to Track & Trace lot record           │ │
    │                  │ └──────────────────────────────────────────────────┘ │
    │                  │                              │                        │
    │ ◄─ status ──────│ (rolling)                    │                        │
```

A key property: **MES drives one handshake at a time per device**, but
**the line is pipelined** — Dispense can be working on cup N+2 while
Weigh works on cup N. MES holds one open `PartId` correlation context
per cup in flight; the data flow matrix (§8.1) tracks N independent
handshakes simultaneously.

### 11.6 Sort decision — synchronous request/response

Sort is the one station where the handshake is inverted: the device
asks MES for a decision, not the other way around. This is also the
**tightest** real-time path in the line.

```
   SORT (DEVICE)                                   MES
        │                                            │
        │  cup arrives, photo-eye triggers           │
        │                                            │
        │  publish:                                  │
        │   Decision/Pending/PartId = "<id>"         │
        │   Decision/Pending/RequestedAtMs = now     │
        │ ──────────────────────────────────────────►│  read; join with
        │                                            │  Weigh.Quality.Last,
        │                                            │  Vision.Inspect.Result
        │                                            │  for this PartId
        │                                            │
        │                                            │  compute bin
        │  ◄──────────────────────── DCMD ──────────│  write:
        │   Decision/Pending/Bin = "A" (or "B")      │   Decision/Pending/Bin
        │                                            │
        │  actuate diverter                          │
        │  publish:                                  │
        │   Decision/Last/{PartId,Bin,Reason}        │
        │ ──────────────────────────────────────────►│  audit
```

**Timeout & fallback** (`Recipe/Parameters/DecisionTimeout`, default
200 ms):

If MES does not write `Decision/Pending/Bin` within the timeout, Sort
falls back to `Recipe/Parameters/RejectBin` and raises **alarm 6003
(MESDecisionTimeout)** with `Decision/Last/Reason="Timeout"`. The cup
goes to reject and the alarm is journalled for engineering review.

MES is expected to write within ~50 ms in steady state. The 200 ms
budget exists for transient broker pauses, not as the working SLA.

### 11.7 Alarm acknowledge handshake

Alarms are bidirectional: the device raises, the operator acks (via
SCADA), the device confirms ack.

```
   DEVICE                          SCADA / OPERATOR
       │                                  │
       │  fault detected                  │
       │  publish (DDATA):                │
       │   Alarm/Active/3003/State=2      │
       │   Alarm/Active/3003/Priority=3   │
       │   Alarm/Active/3003/Message="MassOutOfTolerance"
       │   Alarm/Active/3003/OnsetMs=now  │
       │   Alarm/Summary/UnacknowledgedCount++
       │   (PackML may transition to Holding/Aborting depending on priority)
       │ ────────────────────────────────►│  Ignition alarm pipeline displays
       │                                  │
       │                                  │  operator clicks Acknowledge on HMI
       │  ◄────────────────── DCMD ──────│   Cmd/Alarm/Acknowledge = 3003
       │                                  │
       │  update:                         │
       │   Alarm/Active/3003/State=3      │
       │   Alarm/Active/3003/AckMs=now    │
       │   Alarm/Active/3003/AckUser="alice@plant"
       │   Alarm/Summary/UnacknowledgedCount--
       │ ────────────────────────────────►│  Ignition alarm pipeline updates
       │                                  │
       │  condition clears (physical)     │
       │  update:                         │
       │   Alarm/Active/3003/State=4 (RTN)│
       │ ────────────────────────────────►│  shown as "RTN" in summary
       │  (after retention period)        │
       │   Alarm/Active/3003/State=1 (Normal — entry removed)
```

Critical-priority alarms (Priority=1) automatically transition the
device to **Aborting → Aborted**. The operator must:

1. Resolve the physical condition.
2. Acknowledge: `Cmd/Alarm/Acknowledge = <code>`.
3. Clear the alarm summary: `Cmd/Alarm/Reset = true`.
4. Re-enable production: `Cmd/CntrlCmd = 9 (Clear)` → device goes
   `Aborted → Clearing → Stopped`.
5. Reset to Idle: `Cmd/CntrlCmd = 1 (Reset)` → `Stopped → Resetting →
   Idle`.

This is a deliberate two-step (Clear, then Reset) — Sparkplug-level
matches the OMAC PackML spec exactly and prevents accidental reset
from Aborted by a fat-fingered SCADA write.

### 11.8 Startup & reconnect handshake

```
   DEVICE                  BROKER                  IGNITION (PrimaryHost)
      │                       │                            │
      │                       │   STATE/<H>={online:true}  │
      │                       │ retained ◄─────────────────│  (published on Ignition startup)
      │                       │                            │
      │  set LWT = NDEATH     │                            │
      │  connect (TLS)        │                            │
      │ ─────────────────────►│                            │
      │  subscribe NCMD,DCMD,STATE/<H>                     │
      │  read retained STATE/<H>                           │
      │                                                    │
      │  if PrimaryHost offline:                           │
      │    enter Safe State; PackML=Aborting; alarm 7003   │
      │    do NOT publish DBIRTH                           │
      │  else:                                             │
      │    publish NBIRTH (seq=0, bdSeq=N)                 │
      │    publish DBIRTH for every device                 │
      │ ─────────────────────────────────────────────────► │  consume; refresh UDT
      │                                                    │
      │  start telemetry timer (10 Hz RBE)                 │
```

After reconnect (broker temporarily lost):

- LWT (NDEATH) was already published by the broker when the device
  dropped → SCADA marked all tags STALE.
- Device reconnects, publishes new NBIRTH (bdSeq incremented). SCADA
  drops the stale cache and re-syncs.
- Any cycle that was in flight at disconnect is **not** auto-resumed.
  The device is in `Aborted` (because PrimaryHost-offline triggered
  Safe State on disconnect). Operator must manually clear.

### 11.9 Failure & abort patterns

What happens when the handshake breaks at each step.

| Failure scenario                            | Detector              | Response                                          |
|---------------------------------------------|-----------------------|---------------------------------------------------|
| Device doesn't transition Idle → Execute    | MES timeout (step 3)  | MES writes `Cmd/CntrlCmd=8 (Abort)`; raises alarm |
| Cycle exceeds expected duration             | MES timeout (step 5)  | MES writes `Cmd/CntrlCmd=8 (Abort)`; alarm 7001/9004 |
| Device raises critical alarm mid-cycle      | DDATA on `Alarm/Active/*` | Device auto-aborts; MES marks cup faulted     |
| Sort MES decision timeout                   | Device watchdog       | Sort routes to RejectBin; alarm 6003              |
| Vision inspection timeout                   | MES timeout           | MES routes that cup to RejectBin; alarm 5xxx      |
| Bin full mid-production                     | Device alarm 6001     | MES halts upstream stations until empty; doesn't abort line |
| Broker disconnect (device side)             | TCP / paho            | LWT fires → NDEATH → SCADA marks stale; device reconnects, stays in Aborted |
| PrimaryHost (SCADA) goes offline            | `STATE/<H>=false`     | All devices enter Safe State (Aborting → Aborted); alarm 7003 |
| MES write to non-ready device               | Pre-flight (§11.2 #1) | MES refuses to issue Cmd; logs reason             |
| MES write while `Status/Remote=false`       | Device                | Write ignored; device logs and publishes alarm 9005 |
| Multiple MES Start writes within one cycle  | Device                | Second + are ignored while `_trigger_active`; logged at WARN |
| Stale RBE — device unresponsive             | SCADA heartbeat watchdog | Mark device BAD after 3 s no `Status/Heartbeat` toggle |

### 11.10 Concrete sequence — one cup, end to end

A consolidated trace of every message exchanged for one normal-path
cup, with timing budget:

```
   t (ms)   Actor       Action                                              Target
   ────────────────────────────────────────────────────────────────────────────────
       0    Conveyor    S1 rising edge                                      —
       5    MES         read Position/Sensor/S1                             —
      10    MES         pre-flight check on Dispense                        —
      20    MES         mint PartId, write Material/PartId on Dispense      DCMD
      25    MES         write Cmd/CntrlCmd=2 (Start)                        DCMD → Dispense
      50    Dispense    State 3→2 (Starting)                                DDATA
     100    Dispense    State 2→5 (Execute), CurrentStep=2 (Acknowledged)   DDATA
     150    Dispense    Cylinder/Extended=true                              DDATA
     350    Dispense    Cylinder/Retracted=true                             DDATA
     400    Dispense    State 5→15 (Completing) → 16 (Complete)             DDATA
     400    Dispense    Handshake/JobComplete=true; Counters/Good++         DDATA
     410    MES         observe completion                                  —
     420    MES         write Cmd/CntrlCmd=1 (Reset)                        DCMD → Dispense
     450    Dispense    State 16→14 (Resetting) → 3 (Idle)                  DDATA
     500    Conveyor    belt advances, cup moves toward Fill
    1000    Conveyor    S2 rising edge (AtFill)                             —
     ...    Fill        analogous handshake (~2000 ms cycle)
    3500    Conveyor    S3 rising edge (AtWeigh)
     ...    Weigh       handshake (~1500 ms, settling)
    5500    Conveyor    S4 rising edge (AtVision)
     ...    Vision      handshake (~500 ms, inspection)
    6500    Conveyor    S5 rising edge (AtSort)
    6505    Sort        publish Decision/Pending/PartId
    6550    MES         write Decision/Pending/Bin = "A" (Pass band)
    6580    Sort        actuate diverter → BinA
    6700    Sort        publish Decision/Last; Bin/A/Count++
    6710    MES         finalise Track & Trace record for this PartId
    ────────────────────────────────────────────────────────────────────────────────
   total ≈ 6.7 s per cup (six stations, pipelined → ~1.5 s throughput)
```

Pipelining: while cup N is at Sort (t=6500), cup N+1 is at Vision,
N+2 at Weigh, N+3 at Fill, N+4 at Dispense, N+5 at the conveyor entry.
MES holds six open `PartId` correlation contexts simultaneously.

---

## 12. Unity 3D digital twin integration

Unity is a **read-only Sparkplug subscriber** for digital-twin
visualisation. It connects with publish-denied credentials.

### 11.1 Required subscriber

| Item                          | Value                                          |
|-------------------------------|------------------------------------------------|
| Sparkplug client library      | Eclipse Tahu .NET or MQTTnet + custom decoder  |
| Subscribed topics             | `spBv1.0/<G>/+/+/+` (NBIRTH/DBIRTH/DDATA only) |
| Authentication                | TLS client cert with role `Visualiser`         |
| Broker ACL                    | publish: deny all; subscribe: allow group      |

Unity must implement the Sparkplug consumer behaviours:

- Cache the alias→name map from BIRTH messages.
- Reset cache on NDEATH or BIRTH bdSeq change.
- Mark all metrics under a node as STALE on NDEATH.
- Re-request rebirth (via Unity's own DCMD client would be denied;
  rely on Ignition's Primary Host to manage rebirth instead).

### 11.2 Tags Unity consumes

Visualisation needs **state** (for visual indicators) and **kinematics**
(for actually moving the 3D models).

| Visualisation element              | Tags                                                                |
|------------------------------------|---------------------------------------------------------------------|
| Robot pose (full kinematic chain)  | `Robot.Motion.Joint.J1..J6.Actual.Position`                         |
| Robot EE marker                    | `Robot.Motion.EndEffector.Actual.X/Y/Z/Roll/Pitch/Yaw`              |
| Gripper opening animation          | `Robot.Gripper.Opening.Actual`                                      |
| Robot indicator color              | `Robot.Status.State.Current` (Green=Execute, Yellow=Held, Red=Aborted) |
| Conveyor belt speed                | `Conveyor.Motor.Speed.Actual`                                       |
| Cup positions on belt              | `Conveyor.Position.Sensor.S1..Sn.Triggered` + `CupTracking/<id>/Zone` |
| Dispense cylinder animation        | `Dispense.Cylinder.Position`                                        |
| Fill extruder rotation             | `Fill.Motor.Speed.Actual`                                           |
| Weigh shuttle position             | `Weigh.Shuttle.Position.Actual`                                     |
| Scale reading display              | `Weigh.Scale.Mass.Current`                                          |
| Vision result overlay              | `Vision.Inspect.Result.Last.Classification` + `.ImageRef`           |
| Sort diverter position             | `Sort.Diverter.Position.Current`                                    |
| Bin fill level                     | `Sort.Bin.A.Count / .Capacity`                                      |
| Alarm strobe                       | `<any>.Alarm.Summary.HighestPriority`                               |
| Heartbeat indicator                | `<any>.Status.Heartbeat`                                            |

### 11.3 Smoothing & interpolation

DDATA arrives at 10 Hz with RBE — far below visual fps. Unity must
interpolate between samples. For each tracked tag:

1. Maintain a 2-sample ring buffer with timestamps.
2. Per-frame compute `t_norm = (now - sample[0].t) / (sample[1].t - sample[0].t)` clamped to `[0, 1]`.
3. Linearly interpolate scalar values; SLERP for the EE quaternion
   (which Unity should reconstruct from the Roll/Pitch/Yaw tags or
   subscribe to a future quaternion exposure).

A 5 s full-republish keeps tags from drifting after a packet loss.
Unity should treat a tag as STALE if no DDATA for that tag has
arrived in 10 s.

### 11.4 Recommended Unity tag-binding layer

A `SparkplugTagBinding` MonoBehaviour pattern:

```
[SerializeField] string tagPath = "Robot.Motion.Joint.J1.Actual.Position";
[SerializeField] float interpolationWindow = 0.2f;

void Update() {
    var (value, fresh) = SparkplugClient.Sample(tagPath);
    if (!fresh) SetIndicatorStale();
    else ApplyTo(targetTransform);
}
```

Bind one component per joint, one per EE axis, one per state
indicator. Keep the binding declarative — Unity scenes should not
contain Sparkplug client code outside the singleton client.

---

## 13. Alarm rationalisation (ISA-18.2)

Master alarm list. The `Alarm/Active/<code>/` codes published by each
device come from this table. Priority maps to NAMUR NE107 severity.

| Code | Device   | Name                       | Priority | NE107          | Cause                                  | Operator response                       |
|------|----------|----------------------------|----------|----------------|----------------------------------------|-----------------------------------------|
| 1001 | Conveyor | `BeltStopped`              | 1        | Failure        | E-Stop / motor fault / VFD trip        | Inspect, clear, reset                   |
| 1002 | Conveyor | `BeltSlip`                 | 2        | OOS            | Tension low / overload                 | Reduce load, retension belt             |
| 1003 | Conveyor | `SensorMismatch`           | 3        | Function Check | Two sensors triggered simultaneously   | Check sensor alignment                  |
| 2001 | Dispense | `HopperEmpty`              | 1        | Failure        | No cups remain                         | Refill hopper                           |
| 2002 | Dispense | `HopperLow`                | 4        | Maintenance    | Below refill threshold                 | Schedule refill                         |
| 2003 | Dispense | `CylinderStuck`            | 1        | Failure        | Sensor mismatch > 2 s                  | Check pneumatics, free cylinder         |
| 2004 | Dispense | `AirPressureLow`           | 2        | OOS            | Supply below threshold                 | Check compressor                        |
| 3001 | Fill     | `HopperEmpty`              | 1        | Failure        | No pellets                             | Refill                                  |
| 3002 | Fill     | `MotorOverTemp`            | 2        | OOS            | Extruder motor too hot                 | Cool down, check load                   |
| 3003 | Fill     | `MassOutOfTolerance`       | 3        | Function Check | Weigh reported out-of-band             | Check feed, recalibrate                 |
| 3004 | Fill     | `MotorStalled`             | 1        | Failure        | Motor current > limit                  | Inspect for jam                         |
| 4001 | Weigh    | `ScaleUnstable`            | 3        | Function Check | Vibration / draft                      | Identify disturbance                    |
| 4002 | Weigh    | `ShuttleNotHomed`          | 2        | OOS            | Homing failed at startup               | Re-home, check limit switches           |
| 4003 | Weigh    | `CalibrationDriftHigh`     | 4        | Maintenance    | Span deviation > threshold             | Recalibrate                             |
| 4004 | Weigh    | `LoadCellOverload`         | 2        | OOS            | Reading > max range                    | Remove load, check setup                |
| 5001 | Vision   | `CameraOffline`            | 1        | Failure        | No image acquired                      | Check camera, cabling                   |
| 5002 | Vision   | `LowConfidence`            | 3        | Function Check | Inference below threshold              | Inspect part, review model              |
| 5003 | Vision   | `ModelMissing`             | 2        | OOS            | Configured model not loaded            | Reload model                            |
| 5004 | Vision   | `LightingFailure`          | 2        | OOS            | Ring light unresponsive                | Replace                                 |
| 6001 | Sort     | `BinFull`                  | 2        | OOS            | Capacity threshold reached             | Empty bin                               |
| 6002 | Sort     | `DiverterStuck`            | 1        | Failure        | Position mismatch                      | Check pneumatics                        |
| 6003 | Sort     | `MESDecisionTimeout`       | 3        | Function Check | No MES write in DecisionTimeout        | Check MES connection                    |
| 7001 | Robot    | `MotionTimeout`            | 2        | OOS            | 60 s without completion                | Inspect path, restart                   |
| 7002 | Robot    | `MotionFailed`             | 2        | OOS            | Trajectory aborted by controller       | Check workspace, re-home                |
| 7003 | Robot    | `PrimaryHostOffline`       | 1        | Failure        | STATE/<H> reports offline              | Check Ignition gateway                  |
| 7004 | Robot    | `EStopAsserted`            | 1        | Failure        | Mirror of safety E-Stop                | Resolve hazard, reset                   |
| 7005 | Robot    | `GripperPartLost`          | 2        | OOS            | PartPresent → false mid-cycle          | Retrieve part, recover                  |
| 7006 | Robot    | `JointLimitApproach`       | 4        | Maintenance    | Joint near soft limit                  | Plan check                              |
| 9001 | Any      | `MQTTDisconnected`         | 1        | Failure        | Broker connection lost                 | Network check                           |
| 9002 | Any      | `RecipeLoadFailed`         | 2        | OOS            | `Cmd/Recipe/Load` could not resolve    | Verify recipe id                        |

Each alarm is also documented in the **Sepasoft alarm rationalisation
project document** with full cause/consequence/response/audit-trail.

---

## 14. Safety boundary (ISA-84 / IEC 61511)

The Sparkplug bridge does **not** participate in functional safety.

| Function                | Implemented by                | Bridge role        |
|-------------------------|-------------------------------|--------------------|
| Cell E-Stop             | Safety relay, Cat 3 / PLe     | Read-only mirror   |
| Guard door monitoring   | Safety PLC                    | Read-only mirror   |
| Light curtain           | Safety PLC                    | Read-only mirror   |
| Robot safe speed        | Robot controller (SIL2)       | Read-only mirror   |
| Pneumatic dump valves   | Safety PLC                    | None               |
| Air pressure interlock  | Safety PLC                    | Read-only mirror   |

Bridge rules:

1. Must **not** clear a safety alarm via DCMD. Reset is by physical
   button only.
2. `Cmd/CntrlCmd=Reset` resets the **PackML** state machine; it has no
   authority over safety logic.
3. On `Interlock/EStop=true` the bridge transitions PackML to
   `Aborting → Aborted`. Physical motion is already stopped by hardware
   before the Sparkplug message exists.
4. Unity must visually indicate `Interlock/EStop=true` and not allow
   any HMI control until the safety chain is reset.

---

## Appendix A — PackML state codes (ISA-TR88.00.02)

Published as `Status/State/Current`. The `lookup` PropertySet on this
metric carries the JSON `{code: name}` map.

| Code | State          | Acting? | Predecessor of                   |
|------|----------------|---------|----------------------------------|
| 1    | `Stopped`      | no      | Resetting (1→14)                 |
| 2    | `Starting`     | yes     | Execute (2→5)                    |
| 3    | `Idle`         | no      | Starting (3→2)                   |
| 4    | `Suspended`    | no      | Unsuspending (4→13)              |
| 5    | `Execute`      | yes     | Holding/Completing/Suspending/Stopping/Aborting |
| 6    | `Stopping`     | yes     | Stopped (6→1)                    |
| 7    | `Aborting`     | yes     | Aborted (7→8)                    |
| 8    | `Aborted`      | no      | Clearing (8→17)                  |
| 9    | `Holding`      | yes     | Held (9→10)                      |
| 10   | `Held`         | no      | Unholding (10→11)                |
| 11   | `Unholding`    | yes     | Execute (11→5)                   |
| 12   | `Suspending`   | yes     | Suspended (12→4)                 |
| 13   | `Unsuspending` | yes     | Execute (13→5)                   |
| 14   | `Resetting`    | yes     | Idle (14→3)                      |
| 15   | `Completing`   | yes     | Complete (15→16)                 |
| 16   | `Complete`     | no      | Resetting (16→14)                |
| 17   | `Clearing`     | yes     | Stopped (17→1)                   |

Devices may implement a subset; declare yours in `Admin/PackMLSubset`.

## Appendix B — Handshake step codes

Published as `Handshake/CurrentStep`.

| Code | Label          | When                                            |
|------|----------------|-------------------------------------------------|
| 0    | `Idle`         | At rest                                         |
| 1    | `Triggered`    | Cycle requested, not yet acknowledged           |
| 2    | `Acknowledged` | Bridge has accepted the cycle                   |
| 3    | `Executing`    | Work in progress                                |
| 4    | `Complete`     | Cycle done, `Job_Complete` latched              |
| 9    | `Aborted`      | Fault during cycle                              |

## Appendix C — PhaseState codes (ISA-S88)

Published as `Recipe/Phase/CurrentState`.

| Code | State        |
|------|--------------|
| 1    | `Idle`       |
| 2    | `Running`    |
| 3    | `Complete`   |
| 4    | `Pausing`    |
| 5    | `Paused`     |
| 6    | `Holding`    |
| 7    | `Held`       |
| 8    | `Stopping`   |
| 9    | `Stopped`    |
| 10   | `Aborting`   |
| 11   | `Aborted`    |
| 12   | `Resetting`  |
| 13   | `Restarting` |

## Appendix D — `Cmd/CntrlCmd` codes (OMAC)

Written as `Cmd/CntrlCmd`. The device executes the corresponding
PackML transition and clears the command back to 0.

| Code | Command         | Effect                            |
|------|-----------------|-----------------------------------|
| 0    | `Undefined`     | no-op                             |
| 1    | `Reset`         | Stopped → Idle (via Resetting)    |
| 2    | `Start`         | Idle → Execute (via Starting)     |
| 3    | `Stop`          | * → Stopped (via Stopping)        |
| 4    | `Hold`          | Execute → Held (via Holding)      |
| 5    | `Unhold`        | Held → Execute (via Unholding)    |
| 6    | `Suspend`       | Execute → Suspended (via Suspending) |
| 7    | `Unsuspend`     | Suspended → Execute               |
| 8    | `Abort`         | * → Aborted (via Aborting)        |
| 9    | `Clear`         | Aborted → Stopped (via Clearing)  |
| 10   | `StateComplete` | Force * → next state (used for SC transitions) |

## Appendix E — Unit mode codes (OMAC)

Published as `Status/UnitMode/Current`, written as
`Cmd/UnitMode/Requested`.

| Code | Mode          | Notes                                            |
|------|---------------|--------------------------------------------------|
| 1    | `Production`  | Normal run; only Sepasoft has write authority    |
| 2    | `Maintenance` | Direct phase control allowed                     |
| 3    | `Manual`      | Jog / manual moves enabled                       |
| 4    | `Cleaning`    | CIP / wipe-down state                            |

## Appendix F — Alarm state codes (ISA-18.2)

Published as `Alarm/Active/<code>/State`.

| Code | State          | Meaning                                          |
|------|----------------|--------------------------------------------------|
| 1    | `Normal`       | Condition not present (rarely published — sparse)|
| 2    | `Unack`        | Active and unacknowledged                        |
| 3    | `Ack`          | Active and acknowledged                          |
| 4    | `RTN`          | Returned to normal, awaiting ack-out             |
| 5    | `Shelved`      | Operator-suppressed temporarily                  |
| 6    | `Suppressed`   | Suppressed by program (e.g. during Maintenance)  |

## Appendix G — NAMUR NE107 severity mapping

| Priority (ISA-18.2) | NE107 category         | HMI indicator       |
|---------------------|------------------------|---------------------|
| 1                   | Failure                | Red, blink, audible |
| 2                   | Out-of-Specification   | Orange              |
| 3                   | Function Check         | Yellow              |
| 4                   | Maintenance Required   | Blue                |
| —                   | Good / Normal          | Green               |

---

## Appendix H — Tag-path quick reference

Complete enumeration of every tag path, by device, for code generators
and SCADA importers. Paths shown without the
`spBv1.0/<G>/<N>/<D>/` prefix.

### Conveyor — `Packaging_PLC / Conveyor`

```
Admin/{EquipmentId,EquipmentName,EquipmentClass,Site,Area,Cell,
       Version/{Firmware,Application,TagSpec},PackMLSubset,
       StopReason,MachDesignSpeed}
Status/{State/{Current,CurrentName,Requested},
        UnitMode/{Current,CurrentName},Remote,
        MachSpeed/{Setpoint,Current},Blocked,Starved,Heartbeat}
Handshake/{StationReady,ExecuteCmd,Processing,JobComplete,CurrentStep}
Recipe/{Active/{Id,Name,Version,LoadedAtMs},
        Phase/{Current,CurrentState,StepIndex,StepCount},
        Parameters/{LineSpeed,EmergencyStopRamp}/{Value,Unit}}
Material/{PartId/{Current,Last},Class,
          Source/{Station,Lane},Destination/{Station,Bin},EnteredAtMs}
Interlock/{EStop,GuardClosed,LightCurtainClear,AirPressureOk,PowerOk,
           Upstream/Robot/{MaterialAvailable,Permit},
           Downstream/Dispense/{Ready,Acknowledged}}
Alarm/{Summary/{ActiveCount,UnacknowledgedCount,HighestPriority,HighestSeverityName},
       Active/{1001,1002,1003}/{State,StateName,Priority,Message,OnsetMs,AckMs,AckUser},
       History/Last10}
Cmd/{CntrlCmd,UnitMode/Requested,MachSpeed/Requested,
     MaterialInterlock/{Blocked,Starved},
     Recipe/{Load,Phase/Request},
     Alarm/{Acknowledge,Shelve,Reset}}
Counters/{Cycles/{Total,Good,Faulted,AverageMs},
          Runtime/Seconds,Downtime/Seconds,SinceReset/Cycles}
Diag/{CPULoad,MemoryUsed,
      Network/{RTTToBroker,LastReconnectMs,ReconnectCount},
      Temperature/Controller,Lifetime/Hours}
Motor/{Speed/{Setpoint,Actual},Running,Direction,Load,Current}
Belt/{TensionOk,SlipDetected,TotalDistance}
Position/{Sensor/{S1..S6}/{Triggered,Name,LastEdgeMs},
          CupTracking/<id>/{Zone,EnteredZoneMs}}
```

### Dispense — `Packaging_PLC / Dispense`

```
(universal folders — see Conveyor above)
Cylinder/{Extended,Retracted,Position,
          StrokeTime/{Last,Average},StrokeCount/Total,AirPressure}
Hopper/{Level/{Percent,Low,Empty},Cups/{Remaining,RefillThreshold}}
Recipe/Parameters/{CupType,StrokeHoldTime,MaxStrokeTime}/{Value,Unit}
Alarm/Active/{2001,2002,2003,2004}/...
```

### Fill — `Packaging_PLC / Fill`

```
(universal folders)
Motor/{Speed/{Setpoint,Actual},Running,Current,Temperature}
Extruder/ScrewPosition
Hopper/{Level/{Percent,Low},MassEstimate}
Dosing/{Target/Mass,Dispensed/{Mass,Time},Error/Mass,WithinTolerance}
Recipe/Parameters/{TargetMass,PelletGrade,ExtruderSpeed,Tolerance}/{Value,Unit}
Alarm/Active/{3001,3002,3003,3004}/...
```

### Weigh — `Packaging_PLC / Weigh`

```
(universal folders)
Shuttle/{Position/{Actual,Target},AtScale,AtConveyor,Moving,HomingComplete}
Scale/{Mass/{Current,Last,Tare},Stable,
       Calibration/{LastTimestampMs,Span,Drift}}
Quality/{Last/{Result,ResultName,Tolerance},Pass/Count,Fail/Count}
Recipe/Parameters/{SettleTime,LowerLimit,UpperLimit,ShuttleSpeed}/{Value,Unit}
Alarm/Active/{4001,4002,4003,4004}/...
```

### Vision — `Packaging_Vision / Vision`

```
(universal folders)
Camera/{Online,Exposure/{Setpoint,Actual},Gain,
        Frame/{Rate,LastTimestampMs}}
Lighting/Ring/{On,Intensity}
Inspect/{Trigger,
         Result/Last/{Classification,ClassCode,Confidence,DefectCount,
                      ImageRef,PartId,TimestampMs}}
Model/{Version,TrainedAtMs,ConfidenceThreshold}
Quality/Window/{PassRate,Throughput}
Recipe/Parameters/{ConfidenceThreshold,ModelVersion,InspectTimeout}/{Value,Unit}
Alarm/Active/{5001,5002,5003,5004}/...
```

### Sort — `Packaging_PLC / Sort`

```
(universal folders)
Diverter/{Position/{Current,Setpoint},Moving,StrokeTime/Last}
Bin/{A,B}/{Name,Count,Capacity,Full,Empty}
Decision/{Pending/{PartId,Bin,RequestedAtMs},
          Last/{PartId,Bin,Reason}}
Recipe/Parameters/{RejectBin,DecisionTimeout,BinFullThreshold}/{Value,Unit}
Alarm/Active/{6001,6002,6003}/...
```

### Robot — `Packaging_Robot / Robot`

```
(universal folders)
Motion/{Joint/{J1..J6}/{Actual/{Position,Velocity,Torque},
                         Target/Position},
        EndEffector/{Actual/{X,Y,Z,Roll,Pitch,Yaw},
                     Target/{X,Y,Z,Roll,Pitch,Yaw}},
        Path/ProgressPercent}
Gripper/{Opening/{Actual,Target},Force/{Setpoint,Actual},PartPresent}
Material/Operation/{Current,StepIndex,TotalSteps}
Recipe/Parameters/{Speed,Acceleration,GripperForce,
                   PickApproachHeight,PlaceApproachHeight}/{Value,Unit}
Alarm/Active/{7001..7006}/...
```

---

## Appendix I — Sparkplug message examples

### NBIRTH (Robot node)

```
Topic:   spBv1.0/PlantA_Line1/NBIRTH/Packaging_Robot
Payload: protobuf — Sparkplug B Payload {
   timestamp: 1715441234567,
   seq: 0,
   metrics: [
     { name: "bdSeq",              datatype: Int64,   long_value: 42 },
     { name: "Node Control/Rebirth", datatype: Boolean, boolean_value: false },
     { name: "Node Control/Reboot",  datatype: Boolean, boolean_value: false },
     { name: "Node Control/Next Server", datatype: Boolean, boolean_value: false },
     { name: "Node Control/Scan Rate",   datatype: Int64, long_value: 100 },
   ],
}
```

### DBIRTH (Robot device — abridged)

```
Topic:   spBv1.0/PlantA_Line1/DBIRTH/Packaging_Robot/Robot
Payload: Payload {
   timestamp: 1715441234600,
   seq: 1,
   metrics: [
     { name: "Admin/EquipmentId",          string_value: "AGX-A1B2C3" },
     { name: "Admin/EquipmentClass",       string_value: "Robot" },
     { name: "Admin/Version/TagSpec",      string_value: "2.0" },
     { name: "Status/State/Current",       int_value: 3,    # Idle
       properties: { keys: ["lookup","engUnit","displayName"],
                     values: [<json>,"","Current PackML State"] } },
     { name: "Motion/Joint/J1/Actual/Position", float_value: 0.0,
       properties: { keys: ["engUnit","engLow","engHigh","displayName"],
                     values: ["deg",-180,180,"Joint 1 Angle"] } },
     ...
   ],
}
```

### DDATA (RBE — only changed metrics)

```
Topic:   spBv1.0/PlantA_Line1/DDATA/Packaging_Robot/Robot
Payload: Payload {
   timestamp: 1715441234700,
   seq: 7,
   metrics: [
     { name: "Motion/Joint/J1/Actual/Position", float_value: 12.4,
       timestamp: 1715441234688 },
     { name: "Motion/Joint/J2/Actual/Position", float_value: -5.1,
       timestamp: 1715441234689 },
   ],
}
```

### DCMD (SCADA writes Start)

```
Topic:   spBv1.0/PlantA_Line1/DCMD/Packaging_Robot/Robot
Payload: Payload {
   timestamp: 1715441235000,
   metrics: [
     { name: "Cmd/CntrlCmd", int_value: 2 },   # Start
   ],
}
```

The bridge echoes the value back as DDATA with `is_transient=true`.

---

## Appendix J — Version history

| Version | Date       | Changes                                                              |
|---------|------------|----------------------------------------------------------------------|
| 1.0     | initial    | Robot-only Sparkplug bridge spec                                     |
| 2.0     | 2026-05-11 | Full line refactor — ISA-88/95/TR88.00.02/18.2 alignment, all six stations, OMAC PackTags adoption, Sepasoft + Unity integration |
