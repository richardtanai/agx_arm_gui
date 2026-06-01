Implement the following changes to the topics such taht 


| ISA-95 Level | Value | SPB Field |
|---|---|---|
| Enterprise | `DMATDTS` | Group ID |
| Area | `DLSU` | Edge Node ID |
| Site | `LS` | Device ID |
| Line/Cell/Unit | *(embedded in metric names)* | metric prefix |

**Metric prefix:** `Mini_Factory/camera_node/depthai_camera`

Full DBIRTH topic: `spBv1.0/DMATDTS/DBIRTH/DLSU/LS`

**Primary host (SCADA):** `IgnitionPrimary` — monitored on `spBv1.0/STATE/IgnitionPrimary`

---

## Tag Tree

### Status tags
```
<prefix>/Status/State/Current/Idle        Boolean  one-hot state
<prefix>/Status/State/Current/Execute     Boolean
<prefix>/Status/State/Current/Complete    Boolean
<prefix>/Status/State/Current/Aborted     Boolean
<prefix>/Status/Heartbeat                 Boolean  toggles every heartbeat_interval_s
```

These are one hot state, only one can be active at a time

### Command tags (writable by SCADA → DCMD)
```
<prefix>/Cmd/CntrlCmd/Reset   Boolean  Complete → Idle
<prefix>/Cmd/CntrlCmd/Start   Boolean  Idle → Execute (triggers one detection cycle)
<prefix>/Cmd/CntrlCmd/Stop    Boolean  any → Idle
<prefix>/Cmd/CntrlCmd/Clear   Boolean  Aborted → Idle (clears active alarms first)
```

These are one hot commands, only one can be active at a time
