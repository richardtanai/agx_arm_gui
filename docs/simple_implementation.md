> **DEPRECATED** — superseded by [spb_node_spec.md](spb_node_spec.md). Do not update this file.

# SCADA Operator's Guide — AGX Arm Bridge (ISA-95 minimum)

How to operate the Piper arm from Ignition (or any Sparkplug B-aware SCADA) using
the tags exposed by `spb_bridge_node.py`. Covers connection, normal cycle,
recovery from faults, and disconnect handling.

This guide describes only the **minimum** tag surface implemented in the
migration to ISA-95. For the full design contract see `MQTT-Design-Specs-Full-V2.md`.

---

## 1. Connection summary

### 1.1 Sparkplug identifiers

| Field | Value |
| :---- | :---- |
| Namespace | `spBv1.0` |
| Group ID | `DMATDTS` |
| Edge Node ID | `DLSU` |
| Device ID | `LS` |
| Primary Host ID | `IgnitionPrimary` (configurable) |

The full tag path is therefore:

```
spBv1.0/DMATDTS/{DDATA|DBIRTH|DCMD|...}/DLSU/LS
```

### 1.2 Topics SCADA must subscribe to

| Topic | Purpose |
| :---- | :---- |
| `spBv1.0/DMATDTS/NBIRTH/DLSU` | Node birth — capture once, refresh on every NBIRTH |
| `spBv1.0/DMATDTS/NDEATH/DLSU` | Node death (LWT) — mark all tags stale |
| `spBv1.0/DMATDTS/DBIRTH/DLSU/LS` | Device birth — authoritative metric list |
| `spBv1.0/DMATDTS/DDATA/DLSU/LS` | Live telemetry (RBE) |

### 1.3 Topic SCADA must publish on

| Topic | Direction | Purpose |
| :---- | :---- | :---- |
| `spBv1.0/DMATDTS/DCMD/DLSU/LS` | SCADA -> bridge | Writes to `Cmd/CntrlCmd/*` and `Cmd/TargetID` |
| `spBv1.0/STATE/IgnitionPrimary` | Primary Host -> bridge | Retained `{"online": true}` / `{"online": false}` |

### 1.4 Ignition setup

1. **MQTT Engine** module installed and pointed at the broker.
2. **Primary Host** enabled with `Primary Host ID = IgnitionPrimary`.
3. Bridge tags appear under: `[MQTT Engine]DMATDTS/DLSU/LS/Mini_Factory/agx_arm_bridge/piper_arm/...`
4. Recommended: build a facade UDT under `[default]Mini_Factory/agx_arm_bridge/piper_arm` with reference tags
   into the MQTT Engine tree so HMI screens and scripts only see the ISA-95 path.

---

## 2. The tags that matter

Every cycle is driven by writes to the Cmd tags and observed via the Status tags plus the alarm summary.

### 2.1 Command tags (SCADA -> bridge, all Boolean)

Write `true` to trigger a command. The bridge echoes `false` back after acting.

| Tag (under `...piper_arm/`) | Valid from state | Effect |
| :---- | :---- | :---- |
| `Cmd/CntrlCmd/Start` | Idle | Begin waypoint playback for `Cmd/TargetID` |
| `Cmd/CntrlCmd/Reset` | Complete | Return to Idle after a successful cycle |
| `Cmd/CntrlCmd/Stop` | any | Halt motion and return to Idle immediately |
| `Cmd/CntrlCmd/Clear` | Aborted | Clear all active alarms and return to Idle |
| `Cmd/TargetID` | any | Int32 -- pick which waypoint sequence to play. Persists across cycles. |

### 2.2 Status tags (bridge -> SCADA)

`Status/State/Current/*` tags are **one-hot Booleans**: exactly one is `true` at any moment.

| Tag (under `...piper_arm/`) | True when |
| :---- | :---- |
| `Status/State/Current/Idle` | arm is ready, waiting for a Start |
| `Status/State/Current/Execute` | arm is executing a waypoint sequence |
| `Status/State/Current/Complete` | sequence finished cleanly; awaiting Reset |
| `Status/State/Current/Aborted` | fault detected; awaiting Clear |
| `Status/Heartbeat` | toggles every 0.5 s -- confirms bridge is alive |
| `Alarm/Summary/ActiveCount` | Int32 -- number of active alarms (must be 0 before Start) |

---

## 3. The normal cycle — start to finish

### 3.1 Pre-flight check (SCADA-side, before pressing Start)

Read these values. **All conditions must be true** or the operator must fix something first:

```
Status/State/Current/Idle     == true
Status/Heartbeat               toggling within last 3 s
Alarm/Summary/ActiveCount     == 0
{quality of every read}       == GOOD  (not STALE / BAD)
```

If any fail, do not write `Cmd/CntrlCmd/Start = true`. The bridge will reject it anyway and log a warning.

### 3.2 Six-step cycle

```
   SCADA                                                ROBOT BRIDGE
     |                                                       |
1.   |  (optional, only when changing target)                |
     |  DCMD: Cmd/TargetID = 2                               |
     | ----------------------------------------------------->|  _target_id = 2
     | <---- DDATA: Cmd/TargetID = 2 (echo) ----------------|
     |                                                       |
2.   |  PRE-FLIGHT (read tags, see section 3.1)              |
     |                                                       |
3.   |  DCMD: Cmd/CntrlCmd/Start = true                      |
     | ----------------------------------------------------->|  -> publishes iiot/execute(target=2)
     |                                                       |     starts 60 s watchdog
     | <---- DDATA: Status/State/Current/Execute = true -----|
     | <---- DDATA: Cmd/CntrlCmd/Start = false (echo) -------|
     |                                                       |
4.   |  MONITOR (continuously, until Complete)               |
     |   - Motion/Joint/J1..J6/Actual/Position updates       |
     |   - Gripper/Opening/Actual updates                    |
     |   - Status/Heartbeat keeps toggling                   |
     |                                                       |
5.   | <---- DDATA: Status/State/Current/Complete = true ----|  motion finished cleanly
     |       (PackML latches Complete -- no timeout pressure)|
     |                                                       |
6.   |  DCMD: Cmd/CntrlCmd/Reset = true                      |
     | ----------------------------------------------------->|
     | <---- DDATA: Status/State/Current/Idle = true --------|
     | <---- DDATA: Cmd/CntrlCmd/Reset = false (echo) -------|
     |                                                       |
     |   ready for next cycle                                |
```

### 3.3 Pseudocode an Ignition script could run

```python
def run_one_cycle(target_id: int, timeout_s: float = 90):
    # 1. Pre-flight
    assert robot.Status.State.Current.Idle,          "robot not Idle"
    assert robot.Alarm.Summary.ActiveCount == 0,     "active alarms"
    assert robot.Status.Heartbeat.quality == GOOD,   "robot stale"

    # 2. Select target (optional if already set)
    if robot.Cmd.TargetID != target_id:
        robot.Cmd.TargetID = target_id
        wait_until(robot.Cmd.TargetID == target_id, 2)

    # 3. Start
    robot.Cmd.CntrlCmd.Start = True
    wait_until(robot.Status.State.Current.Execute,  2)           # Execute
    wait_until(robot.Status.State.Current.Complete, timeout_s)   # Complete

    # 4. Reset back to Idle
    robot.Cmd.CntrlCmd.Reset = True
    wait_until(robot.Status.State.Current.Idle, 2)               # Idle
```

That is the entire happy-path contract.

### 3.4 What if `wait_until` for Complete times out?

The robot is stuck in Execute longer than the recipe allows. SCADA should:

1. Write `Cmd/CntrlCmd/Stop = true` -- bridge halts motion and returns to Idle.
2. Treat the cycle as failed for MES/recipe purposes.
3. Investigate logs from the bridge (`/iiot/status` traffic, ROS controller diagnostics).

The bridge has its own 60 s watchdog for motion that never completes -- see section 5 (Alarm 7001).

---

## 4. The PackML state machine — quick reference

The bridge implements only the four PackML states needed for the minimum:

```
                 Cmd/CntrlCmd/Start = true
        +--------------------------------------+
        v                                      |
   +---------+      motion complete       +---------+
   |  Idle   | --------------------------->| Execute |
   +---------+                             +---------+
        ^                                       |
        | Cmd/CntrlCmd/Reset = true             | motion complete
        |                                       v
        |                                  +----------+
        +----------------------------------|  Complete |
        |                                  +----------+
        |
        | Cmd/CntrlCmd/Clear = true
        |
   +---------+
   | Aborted |  <--- (entered automatically on fault, see section 5)
   +---------+
```

| Command (write `true`) | Valid from | Effect |
| :---- | :---- | :---- |
| `Cmd/CntrlCmd/Start` | Idle | -> Execute (begins waypoint playback for `Cmd/TargetID`) |
| `Cmd/CntrlCmd/Reset` | Complete | -> Idle |
| `Cmd/CntrlCmd/Stop` | any | halt motion, -> Idle |
| `Cmd/CntrlCmd/Clear` | Aborted | clear active alarms, -> Idle |

Writes from an invalid source state are logged at WARN and ignored. The bridge does not punish bad commands.

---

## 5. Alarms — recognising and recovering

### 5.1 Alarm tags

For each defined alarm code (7001-7006), the bridge publishes four tags:

```
Alarm/Active/<code>/State       Int32   1 = Normal, 2 = Unack
Alarm/Active/<code>/Priority    Int32   1 = Critical, 2 = High, 4 = Maintenance
Alarm/Active/<code>/Message     String  e.g. "MotionTimeout"
Alarm/Active/<code>/OnsetMs     Int64   epoch ms when alarm fired (0 if never)
Alarm/Summary/ActiveCount       Int32   count of codes with State != Normal
```

`Priority` and `Message` are static -- set once at DBIRTH and never change. Only `State` and `OnsetMs`
move at runtime. SCADA's alarm pipeline should bind to `State` and use the rest as metadata.

### 5.2 Alarm catalogue

| Code | Priority | Message | When raised | When cleared |
| :---- | :---- | :---- | :---- | :---- |
| `7001` | 2 High | `MotionTimeout` | 60 s in Execute without completion | `Cmd/CntrlCmd/Clear = true` |
| `7002` | 2 High | `MotionFailed` | ROS controller reports motion error/abort | `Cmd/CntrlCmd/Clear = true` |
| `7003` | 1 Critical | `PrimaryHostOffline` | `STATE/IgnitionPrimary` goes offline | **auto** on `STATE/IgnitionPrimary` -> online |
| `7004` | 1 Critical | `EStopAsserted` | (declared; bridge integration pending) | `Cmd/CntrlCmd/Clear = true` |
| `7005` | 2 High | `GripperPartLost` | (declared; bridge integration pending) | `Cmd/CntrlCmd/Clear = true` |
| `7006` | 4 Maintenance | `JointLimitApproach` | (declared; bridge integration pending) | `Cmd/CntrlCmd/Clear = true` |

### 5.3 What happens on a fault

Bridge-internal sequence when any alarm raises:

1. Active motion is halted (`iiot/halt` published).
2. `Alarm/Active/<code>/State` -> 2 (Unack); `OnsetMs` set.
3. `Alarm/Summary/ActiveCount` incremented.
4. `Status/State/Current/Aborted` -> `true` (all other state booleans -> `false`).

SCADA's alarm pipeline observes the state change and lights up the HMI banner.

### 5.4 Operator recovery sequence

```
   OPERATOR                  SCADA                              ROBOT BRIDGE
        |                       |                                     |
        |                       | <--- State/Current/Aborted = true --|
        |                       | <--- Alarm/.../State = 2 (Unack) --|
        |  fault banner         |                                     |
        | <--------------------|                                     |
        |                       |                                     |
        |  physically resolve   |                                     |
        |  (e.g. E-Stop reset,  |                                     |
        |   retrieve workpiece) |                                     |
        |                       |                                     |
        |  click "Clear" on HMI |                                     |
        | --------------------> |                                     |
        |                       | DCMD: Cmd/CntrlCmd/Clear = true     |
        |                       | ----------------------------------->|
        |                       |                                     |  clears all alarms
        |                       |                                     |  Aborted -> Idle
        |                       | <--- Alarm/.../State = 1 (Normal) --|
        |                       | <--- State/Current/Idle = true -----|
        |  ready to retry       |                                     |
        | <--------------------|                                     |
```

`Clear` is a **bulk operation** in this minimum implementation -- there is no per-code Acknowledge.
All active alarms transition `Unack -> Normal` together.

The single exception is `7003 (PrimaryHostOffline)`: it auto-clears the instant `STATE/IgnitionPrimary`
becomes `online` again. Operators never see it stuck after Ignition recovers.

---

## 6. Stale data & disconnect handling

### 6.1 What SCADA sees when the robot drops

1. **~0.3 s:** Broker publishes `NDEATH` (the LWT the bridge registered before connecting). MQTT Engine
   marks every robot tag as STALE / `Bad_StaleDevice`.
2. **~3 s:** Even if `NDEATH` was lost, `Status/Heartbeat` has stopped toggling. SCADA marks the tags
   BAD via the heartbeat-staleness check independently.
3. **HMI display:** every tag shows its last value but with stripe/grey styling. A state boolean may still
   read `true` for a while, but the quality bit is BAD -- bind your HMI logic on quality, not value.

### 6.2 The bridge reconnects

When the robot bridge comes back:

1. New `NBIRTH` published (fresh `bdSeq`).
2. New `DBIRTH` published. `Status/State/Current/Idle` will be `true` if `STATE/IgnitionPrimary` is
   online, otherwise the bridge enters Safe State (`Status/State/Current/Aborted = true` + alarm `7003`).
3. MQTT Engine drops its stale cache and rebuilds from the DBIRTH. All tags go GOOD again.

**No in-flight cycle is resumed.** Any cup or workpiece that was being handled is left wherever it ended
up; the operator must visually verify and recover.

### 6.3 Pre-flight implication for SCADA

Always check `quality == GOOD` *and* `Status/State/Current/Idle == true` before writing
`Cmd/CntrlCmd/Start = true`. A stale read of `Idle = true` from before a disconnect is meaningless -- the
bridge may currently be offline or recovering, and writes during that window are lost (DCMDs are not queued).

---

## 7. Quick-reference: SCADA write rules

| If you want to... | Write | After it succeeds, expect |
| :---- | :---- | :---- |
| Pick a waypoint sequence | `Cmd/TargetID = <id>` | `Cmd/TargetID` echoes back to `<id>` |
| Start a cycle | `Cmd/CntrlCmd/Start = true` | `Idle->false`, `Execute->true`, ..., `Complete->true` |
| Acknowledge cycle complete | `Cmd/CntrlCmd/Reset = true` | `Complete->false`, `Idle->true` |
| Interrupt a running cycle | `Cmd/CntrlCmd/Stop = true` | motion halts, `Execute->false`, `Idle->true` |
| Recover from fault | (resolve physical cause first) then `Cmd/CntrlCmd/Clear = true` | alarms `State->1`, `Aborted->false`, `Idle->true` |
| Force a fresh DBIRTH refresh | `Node Control/Rebirth = true` (on NCMD topic) | NBIRTH + DBIRTH republished |

### 7.1 What SCADA should *not* do

- **Do not** spam `Cmd/CntrlCmd/Start = true` waiting for state to change. Each write is logged; one is enough.
  PackML latches state until the upstream actor explicitly transitions it.
- **Do not** write to anything outside `Cmd/CntrlCmd/*`, `Cmd/TargetID`, or `Node Control/Rebirth`. Other
  writes are logged as `DCMD ignored (unknown metric)` and have no effect.
- **Do not** rely on `Cmd/CntrlCmd/*` value as state -- the bridge echoes `false` after acting. Use
  `Status/State/Current/*` for state, always.
- **Do not** issue `Start` while the cycle is still in `Complete`. Either `Reset` first (-> Idle),
  or the bridge will refuse the new Start.

---

## 8. Worked example — running waypoint sequence #2 (pick.yaml)

Step-by-step what an operator clicking "Run Pick" on the HMI causes on the wire:

```
T+0.0s   HMI "Run Pick" click
T+0.0s   SCADA: DCMD Cmd/TargetID = 2
T+0.1s   bridge: DDATA Cmd/TargetID = 2 (echo)
         bridge logs: "DCMD Cmd/TargetID: 0 -> 2"
T+0.2s   SCADA: pre-flight passes
         SCADA: DCMD Cmd/CntrlCmd/Start = true
T+0.3s   bridge logs: "DCMD Cmd/CntrlCmd/Start=True | state=3 (Idle) | target_id=2"
         bridge publishes iiot/execute(target=2) onto ROS
         bridge: DDATA Status/State/Current/Execute = true  (Idle = false)
         bridge: DDATA Cmd/CntrlCmd/Start = false  (echo)
T+0.3s.. HMI shows "Running"; joint angles and gripper opening update at 10 Hz (RBE)
T+8.4s   WaypointManager publishes iiot/status "complete:2"
         bridge: DDATA Status/State/Current/Complete = true  (Execute = false)
T+8.4s   HMI shows "Complete -- press Reset to continue"
T+9.5s   operator clicks "Reset"
         SCADA: DCMD Cmd/CntrlCmd/Reset = true
         bridge: DDATA Status/State/Current/Idle = true  (Complete = false)
         bridge: DDATA Cmd/CntrlCmd/Reset = false  (echo)
         HMI shows "Idle -- ready"
```

Total wire traffic for one cycle: 2 DCMDs and roughly 80-150 DDATA messages (joint RBE + heartbeat).

---

## 9. Where to look when things don't work

| Symptom | First thing to check |
| :---- | :---- |
| `Cmd/CntrlCmd/Start` ignored | Check bridge log for "Start ignored: state must be Idle". Likely `Status/State/Current/Idle` is `false` -- perhaps a previous Complete was never Reset. |
| Cycle never finishes | `iiot/status` from WaypointManager -- is it publishing `complete:<id>`? If WaypointManager is not listening on `iiot/execute`, the bridge prints "iiot/execute has NO subscribers". |
| Alarms keep firing on connect | `STATE/IgnitionPrimary` may not be retained `{online: true}`. Verify the Ignition Primary Host is enabled and the broker is preserving the retained STATE message. |
| Tags stay BAD even after the bridge prints "MQTT connected" | Check broker ACL for the bridge's MQTT user -- needs publish on `spBv1.0/DMATDTS/#`. |
| `Cmd/TargetID` write seems to do nothing | Read it back via DDATA after the write. If echo doesn't match, the DCMD didn't reach the bridge (broker / topic / cert issue). |
