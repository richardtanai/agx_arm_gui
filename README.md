# agx_arm_gui

A PyQt5 dashboard for the Agilex Piper arm. Manages CAN, the arm controller, MoveIt2, and a Sparkplug B bridge — and lets you record/replay waypoint sequences either by hand or as an **IIoT slave device** triggered by a SCADA primary host.

---

## What it does

| Panel | Purpose |
|---|---|
| **CAN Bus Management** | Brings up the CAN interface via `can_activate.sh` and watches `/sys/class/net/<iface>/operstate`. |
| **Launch Control** | Starts/stops the arm controller, MoveIt2, and the Sparkplug B bridge with the right launch arguments. |
| **Waypoint Recorder & Playback** | Snapshots `feedback/joint_states` + `feedback/gripper_status` into a YAML file, replays it through `control/joint_states` with a 0.10×–2.00× speed slider. |
| **IIoT Device Mode** | Toggle inside the Waypoint panel. When ON, the GUI plays back a sequence in response to a Sparkplug B `Commands/Trigger` from a SCADA host (Ignition, etc.) and reports completion via the Standard Handshake (`Job_Complete`). |
| **Log Console** | Aggregates stdout/stderr from every managed process. |

---

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- The `agx_arm_ros` workspace built and sourced (it provides the arm controller, MoveIt config, and `agx_arm_msgs/GripperStatus`)
- System packages: `python3-pyqt5`, `python3-yaml`, `python3-paho-mqtt`
- Eclipse Tahu Python bindings on the `PYTHONPATH` (`tahu.sparkplug_b`, `tahu.sparkplug_b_pb2`)
- An MQTT broker reachable from the host (Mosquitto, HiveMQ, the broker embedded in Ignition, …)

---

## Build

```bash
cd ~/agx_arm_ws
colcon build --packages-select agx_arm_msgs agx_arm_gui
source install/setup.bash
```

---

## Run the GUI

```bash
ros2 run agx_arm_gui agx_arm_gui
```

The window has three tiers: process control on top, **Waypoint Recorder & Playback** in the middle, and the log console at the bottom.

### Configuration

All defaults — broker, identity, deadbands, target map — live in [config/gui_params.yaml](config/gui_params.yaml). Override the file location with:

```bash
export AGX_ARM_CONFIG=/path/to/my_gui_params.yaml
```

---

## Recording & playing back a sequence (manual mode)

1. Bring the CAN bus up (or check **Simulated Mode** in the header).
2. Click **Launch** on the Arm Controller (and optionally MoveIt2).
3. **Click Enable** (next to Launch/Stop on the Arm Controller row). This calls the `/enable_agx_arm` service. *Without this step the controller silently drops every command — the arm will look stuck.*
4. Hand-guide / jog the arm to a pose, then click **Record Waypoint** in the Waypoint panel. Repeat for each pose.
5. Edit the **Hold (s)** column inline if you need a longer dwell on any waypoint.
6. **Save YAML…** writes the sequence to `~/agx_arm_ws/waypoints/` by default.
7. **▶ Play** replays the sequence; **Speed** scales every hold time live; **Stop** cancels.
8. **Move to Home** calls the controller's `/move_home` service so you can park the arm in a known pose between recordings.

### How playback finds the arm

The panel auto-picks the right transport at the moment you press Play:

| Setup | Transport used | Why |
|---|---|---|
| Arm Controller only | Direct publish on `control/joint_states` | The controller subscribes; nothing else is on the topic. |
| Arm Controller **+ MoveIt2** | `/arm_controller/follow_joint_trajectory` action | MoveIt's `joint_state_broadcaster` floods `control/joint_states` at 200 Hz, so a direct publish gets drowned out. The action goes straight through `joint_trajectory_controller` and the controller interpolates between waypoints for you. |
| MoveIt sim only (no Arm Controller) | `/arm_controller/follow_joint_trajectory` action | Same as above — only the action moves the mock hardware. |

The log line at playback start tells you which one was chosen.

> Press Play and nothing moves? Check the log line. If it says "no way to move the arm", neither transport is available — launch the Arm Controller (and Enable it) or launch MoveIt first.

YAML schema:

```yaml
version: 1
arm_type: piper
joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, gripper]
waypoints:
  - name: wp_1
    positions: [0.0, -0.4, 0.5, 0.0, 0.0, 0.0, 0.5]
    gripper_width: 0.04        # metres; null when no gripper feedback
    hold_time: 1.0
```

---

## IIoT Device Mode (SCADA-triggered playback)

In this mode the robot becomes a slave device: a Sparkplug B primary host sends a Trigger and the GUI plays the corresponding recorded sequence.

### One-time setup

1. Edit [config/gui_params.yaml](config/gui_params.yaml). Point `spb_bridge.local` (or `spb_bridge.hivemq`) at your broker, set `primary_host_id` to the SCADA host's STATE-topic ID (Ignition default is the gateway name), and populate `iiot_device.target_map`:

   ```yaml
   iiot_device:
     waypoints_dir: ~/agx_arm_ws/waypoints
     default_speed: 1.0
     target_map:
       1: pick.yaml
       2: place.yaml
       3: home.yaml
   ```

2. Record + save each named YAML once using the manual flow above.

### Bring-up

1. Activate CAN, launch the Arm Controller (sim or physical).
2. **Launch** the SPB Bridge from the Launch Control panel (host/port/credentials are filled from config).
3. In the Waypoint panel, tick **IIoT Device Mode**. The status label turns green and reads `ready`.

### What happens at runtime

```
SCADA / Ignition                 SPB Bridge (ros2 run)              GUI WaypointManager
      │                                  │                                  │
      │── DCMD Commands/TargetID = 2 ───►│                                  │
      │── DCMD Commands/Trigger  = true ►│ Phase 1 — Triggered              │
      │                                  │ Phase 2 — Acknowledge ──DDATA──► │
      │                                  │ Status/Busy = true               │
      │── DCMD Commands/Trigger  = false►│ Phase 3 — iiot/execute(2) ─────► │
      │                                  │                                  │ play place.yaml
      │                                  │ ◄─ iiot/status started:2         │
      │                                  │ ◄─ iiot/status progress:i/total  │
      │                                  │ ◄─ iiot/status complete:2        │
      │                                  │ Phase 4 ─DDATA─►                 │
      │                                  │ Job_Complete = true (latched)    │
      │── DCMD JobComplete_Ack = false ─►│ clears Job_Complete              │
```

`TargetID = 0` is a commissioning shortcut — it plays whatever sequence is currently loaded in the GUI, so you can record-then-trigger without touching the YAML map.

### What gets published on the wire

- **NBIRTH / DBIRTH** on connect — declares the schema (Status/PackML, Handshake/*, Positions/Joints/J1..J6, Positions/EE/X..Yaw, Commands/*).
- **DDATA** — joint and EE telemetry at 10 Hz with Report-by-Exception (deadbands tunable in `gui_params.yaml`); full re-publish every 5 s as a stale-detection heartbeat; `Status/Heartbeat` toggles every 500 ms.
- **NDEATH** — registered as MQTT Last Will, fires automatically if the bridge crashes or loses the broker.

### Safe State

The bridge subscribes to `spBv1.0/STATE/<primary_host_id>`. If the host goes OFFLINE, the bridge:

1. Halts the active playback (publishes `iiot/halt`).
2. Sets `Status/PackML = Aborted`, `Handshake/Fault_Code = 300` (PrimaryHostOffline).

Bring the SCADA back online and reissue the Trigger to resume.

---

## Topic protocol (between bridge and manager)

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `iiot/execute` | `std_msgs/Int32` | bridge → manager | Target ID to play |
| `iiot/halt`    | `std_msgs/Empty` | bridge → manager | Cancel current playback |
| `iiot/status`  | `std_msgs/String` | manager → bridge | `ready` / `started:N` / `progress:i/t` / `complete:N` / `aborted:N` / `error:msg` |

These are plain ROS topics — you can `ros2 topic echo iiot/status` while the GUI runs to debug a stuck handshake without touching the broker.

---

## Bench-test the SpB handshake without Ignition

```bash
# Terminal A — broker (skip if you already run one)
mosquitto -v

# Terminal B — GUI (records sequences, runs IIoT mode)
ros2 run agx_arm_gui agx_arm_gui
# → Save a sequence as ~/agx_arm_ws/waypoints/pick.yaml
# → Tick "IIoT Device Mode"

# Terminal C — fake SCADA: publish DCMD via mosquitto_pub or Eclipse Tahu CLI
# Easiest: ros2 topic pub the bridge directly
ros2 topic pub --once /iiot/execute std_msgs/Int32 "data: 1"
# Watch /iiot/status to see started:1 → progress:.. → complete:1
```

---

## Files

```
agx_arm_gui/
├── config/gui_params.yaml          # broker, identity, IIoT target_map, deadbands
├── agx_arm_gui/
│   ├── main.py                     # MainWindow + panel composition
│   ├── process_manager.py          # subprocess lifecycle (CAN / arm / MoveIt / SPB)
│   ├── ros_monitor.py              # polls `ros2 node list`
│   ├── waypoint_manager.py         # rclpy node — record / save / load / play / iiot bridge
│   ├── waypoint_panel.py           # Qt panel for waypoints + IIoT toggle
│   ├── spb_bridge_node.py          # Sparkplug B bridge — Tahu, RBE, Standard Handshake
│   └── config_loader.py            # typed view of gui_params.yaml
└── scripts/                        # entry-point shims
```

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| **Pressed Play but arm doesn't move (Arm Controller alone)** | Arm Controller not enabled. Click the **Enable** button next to Launch/Stop on the Arm Controller row — without it the controller drops every command and prints `Agx_arm is not enabled, cannot control` on its own log. |
| **Pressed Play but arm doesn't move (MoveIt running)** | The trajectory action wasn't ready when you clicked. Wait until `ros2 action list` shows `/arm_controller/follow_joint_trajectory` then try again. The log line at playback start should read "via FollowJointTrajectory action". |
| **Pressed Play, panel warns "no way to move the arm"** | Neither transport is up. Launch the Arm Controller (and Enable it) — and/or launch MoveIt — first. |
| **SPB bridge keeps disconnecting/reconnecting** | Multiple `spb_bridge_node` processes are connecting to the broker with the same Sparkplug `client_id` — each new connection kicks the previous one off. The GUI now sweeps these up automatically before launching, but if you see this happen mid-session it usually means another process outside this GUI is running. Verify with `pgrep -af spb_bridge_node` and `pkill -f spb_bridge_node` if needed. |
| `iiot/status` reports `error:no_sequence` | `TargetID` not in `iiot_device.target_map`, or the YAML file is missing under `waypoints_dir`. |
| `Handshake/Fault_Code = 300` after launch | The bridge can reach the broker but never saw `STATE/<primary_host_id> = ONLINE`. Check `primary_host_id` matches the SCADA gateway name. |
| GUI logs "GripperStatus unavailable" | The `agx_arm_msgs` overlay isn't sourced — gripper width won't be recorded but the rest still works. |
| `Job_Complete` stays latched true | SCADA must write `Commands/JobComplete_Ack = false` to clear it (Transactional Integrity guard). |
| Playback overshoots / stutters | Tighten `hold_time` per waypoint or drop the speed slider; the publish rate is fixed at 50 Hz. |

---

## Kill everything the GUI launched

If the GUI crashes or you Ctrl-C the window before stopping its panels, the child processes it spawned (`ros2 launch …` for the arm controller, MoveIt, and `spb_bridge_node`) can survive as orphans. Run these in order — most of the time the first block is enough.

**1. Graceful shutdown by name:**

```bash
pkill -TERM -f 'spb_bridge_node'
pkill -TERM -f 'start_single_agx_arm.launch.py'
pkill -TERM -f 'start_single_agx_arm_moveit.launch.py'
pkill -TERM -f 'agx_arm_moveit.*demo.launch.py'
sleep 2
```

**2. Force-kill survivors:**

```bash
pkill -KILL -f 'spb_bridge_node|start_single_agx_arm|agx_arm_moveit|agx_arm_ctrl_single_node'
```

**3. Sweep leftover ROS 2 / MoveIt children that the launches forked (`robot_state_publisher`, `move_group`, RViz, controllers):**

```bash
pkill -KILL -f 'ros2 (run|launch)'
pkill -KILL -f 'robot_state_publisher|move_group|rviz2|controller_manager|joint_state_broadcaster|ros2_control_node'
```

**4. Verify nothing remains:**

```bash
pgrep -af 'spb_bridge|agx_arm|move_group|rviz2|robot_state_publisher|ros2'
```

If `pgrep` returns nothing, you're clean. For stubborn zombies (state `Z`) find the parent and kill it:

```bash
ps -eo pid,ppid,stat,cmd | awk '$3 ~ /Z/'
```

**One-liner — nuke everything:**

```bash
pkill -KILL -f 'spb_bridge|agx_arm|ros2 (run|launch)|move_group|rviz2|robot_state_publisher|controller_manager|ros2_control_node'
```

> Avoid `killall ros2` or `pkill python3` — those hit unrelated processes. The patterns above are scoped to what `process_manager.py` actually launches.
