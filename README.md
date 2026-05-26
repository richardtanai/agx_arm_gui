# agx_arm_gui

A PyQt5 dashboard for the Agilex Piper arm. Manages CAN, the arm controller, MoveIt2, and a Sparkplug B MQTT bridge — and lets you record/replay waypoint sequences either manually or as an **IIoT device** triggered by a SCADA primary host (Ignition).

---

## What it does

| Panel | Purpose |
|---|---|
| **CAN Bus Management** | Brings up the CAN interface via `can_activate.sh` and watches `/sys/class/net/<iface>/operstate`. |
| **Launch Control** | Starts/stops the arm controller, MoveIt2, and the Sparkplug B bridge with the right launch arguments. |
| **Waypoint Recorder & Playback** | Snapshots `feedback/joint_states` + `feedback/gripper_status` into a YAML file, replays it through `control/joint_states` with a 0.10×–2.00× speed slider. |
| **IIoT Device Mode** | Toggle inside the Waypoint panel. When ON, the GUI plays back a sequence in response to a Sparkplug B DCMD from a SCADA host and reports state via ISA-95 Boolean tags. |
| **Log Console** | Aggregates stdout/stderr from every managed process. |

---

## Installation

### Prerequisites

- **OS:** Ubuntu 22.04 LTS (Jammy)
- **ROS:** ROS 2 Humble (desktop or base install)
- **Hardware:** Agilex Piper arm connected via USB-CAN adapter

---

### Step 1 — System packages (apt)

```bash
sudo apt update
sudo apt install -y \
    can-utils \
    ethtool \
    python3-pyqt5 \
    python3-yaml \
    python3-psutil \
    python3-colcon-common-extensions
```

**ROS 2 control and motion packages:**

```bash
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-topic-tools \
    ros-humble-xacro \
    ros-humble-rviz2
```

**MoveIt2 (required for trajectory action server):**

```bash
sudo apt install -y ros-humble-moveit
```

---

### Step 2 — Python packages (pip, system-level)

> **Why system-level?** The ROS node (`spb_bridge_node`) is launched by the GUI as a subprocess that inherits the system Python, not a virtualenv. All three packages must be visible to `python3` without any environment activation.

```bash
pip3 install --break-system-packages \
    "paho-mqtt>=2.0" \
    tahu \
    python-can \
    scipy \
    numpy
```

| Package | Used by | Purpose |
|---|---|---|
| `paho-mqtt>=2.0` | `spb_bridge_node.py` | MQTT client (Sparkplug B transport) |
| `tahu` | `spb_bridge_node.py` | Eclipse Tahu Sparkplug B protobuf bindings |
| `python-can` | `agx_arm_ctrl` | CAN bus communication with the arm hardware |
| `scipy` | `agx_arm_ctrl` | Kinematics / trajectory math |
| `numpy` | `agx_arm_ctrl` | Array math support |

> **Note on `tahu`:** the PyPI package (`pip install tahu`) is a community port — it provides `tahu.sparkplug_b` and `tahu.sparkplug_b_pb2`. Do **not** use the Eclipse Tahu GitHub source directly; the PyPI version is what this package imports.

---

### Step 3 — Workspace packages

Clone `agx_arm_ros` into the same workspace as `agx_arm_gui`. It provides the arm controller, MoveIt config, URDF, and the `agx_arm_msgs` message definitions that the bridge depends on.

```bash
cd ~/agx_arm_ws/src
git clone <agx_arm_ros-repo-url> agx_arm_ros
```

The workspace must contain these four packages (all inside `agx_arm_ros/src/`):

| Package | Type | Purpose |
|---|---|---|
| `agx_arm_msgs` | ament_cmake | Custom message definitions (`GripperStatus`, etc.) |
| `agx_arm_ctrl` | ament_python | Arm controller node + CAN driver |
| `agx_arm_description` | ament_cmake | URDF / xacro for the Piper arm |
| `agx_arm_moveit` | ament_cmake | MoveIt2 configuration and launch files |

---

### Step 4 — Build

```bash
cd ~/agx_arm_ws

# Source ROS before building
source /opt/ros/humble/setup.bash

# Build — always build agx_arm_msgs first so the generated Python stubs exist
colcon build --packages-select agx_arm_msgs
colcon build --packages-select agx_arm_ctrl agx_arm_description agx_arm_moveit agx_arm_gui

source install/setup.bash
```

> Add `source ~/agx_arm_ws/install/setup.bash` to your `~/.bashrc` so you do not have to re-source every terminal.

---

### Step 5 — MQTT broker

The Sparkplug B bridge requires an MQTT broker. For local development, Mosquitto is the simplest option:

```bash
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
```

For production or HiveMQ Cloud, see the `hivemq` section of [config/gui_params.yaml](config/gui_params.yaml).

---

### Step 6 — Configuration

**Copy and edit the secrets file:**

```bash
cp ~/agx_arm_ws/install/agx_arm_gui/share/agx_arm_gui/config/gui_secrets.example.yaml \
   ~/agx_arm_ws/src/agx_arm_gui/config/gui_secrets.yaml
```

Edit `gui_secrets.yaml` and fill in your broker host / credentials. This file is gitignored — never commit it.

```yaml
spb_bridge:
  local:
    host: localhost      # or your broker IP
    username: ""
    password: ""
```

**Review the main config** at [config/gui_params.yaml](config/gui_params.yaml). Key fields:

```yaml
can:
  interface: can0                  # CAN interface name (check with: ip link show | grep can)

arm_controller:
  arm_type: piper                  # piper | piper_x | piper_l | piper_h | nero
  effector_type: agx_gripper       # none | agx_gripper | revo2

spb_bridge:
  group_id: DMATDTS                # Sparkplug B / ISA-95 identity (see docs below)
  edge_node_id: DLSU
  device_id: LS
  primary_host_id: IgnitionPrimary # Must match the Ignition gateway name

iiot_device:
  waypoints_dir: ~/agx_arm_ws/waypoints
  target_map:
    1: pick.yaml
    2: place.yaml
    3: home.yaml
```

Override either file at runtime without rebuilding:

```bash
export AGX_ARM_CONFIG=/path/to/my_gui_params.yaml
export AGX_ARM_SECRETS=/path/to/my_gui_secrets.yaml
```

---

## Running the GUI

```bash
source ~/agx_arm_ws/install/setup.bash
ros2 run agx_arm_gui agx_arm_gui
```

The window opens with three tiers: process control (top), Waypoint Recorder & Playback (middle), and log console (bottom).

---

## Using the GUI

### Bring-up sequence

1. **CAN Bus** row → click **Activate**. The status indicator turns green when `/sys/class/net/can0/operstate` reads `up`. If the indicator stays red, check the USB-CAN adapter is plugged in and run `ip link show can0`.

2. **Arm Controller** row → click **Launch**. Wait until the log shows `Arm Controller ready`.

3. **Arm Controller** row → click **Enable**. This calls the `/enable_agx_arm` service. **The arm silently drops all commands until this is done.**

4. *(Optional)* **MoveIt2** row → click **Launch**. Required if you want RViz visualisation or inverse-kinematics-based motion. Wait for `move_group` to appear in `ros2 node list`.

5. *(Optional)* **SPB Bridge** row → click **Launch** if SCADA/IIoT integration is needed.

---

### Recording and replaying waypoints (manual)

1. Jog the arm to the first pose (hand-guide or jog service).
2. Click **Record Waypoint** — the current joint angles and gripper width are added to the table.
3. Repeat for each pose. Edit the **Hold (s)** column inline to set dwell time at each waypoint.
4. Click **Save YAML…** — saves to `~/agx_arm_ws/waypoints/` by default.
5. Click **▶ Play** to replay. Use the **Speed** slider (0.10×–2.00×) to scale all hold times live.
6. Click **Stop** to cancel playback. Click **Move to Home** to park the arm in the home pose.

**Transport selection at playback:**

| Setup | Transport used |
|---|---|
| Arm Controller only (no MoveIt) | Direct publish on `control/joint_states` |
| Arm Controller + MoveIt2 | `/arm_controller/follow_joint_trajectory` action |
| MoveIt2 only (sim) | `/arm_controller/follow_joint_trajectory` action |

The log line at playback start tells you which transport was chosen. If it says "no way to move the arm", launch the Arm Controller (and Enable it) first.

**Waypoint YAML schema:**

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

### IIoT Device Mode (SCADA-triggered playback)

Enable by ticking **IIoT Device Mode** in the Waypoint panel. The status label turns green and reads `ready`.

The bridge node then listens for Sparkplug B DCMDs from the SCADA primary host:

| DCMD write | Effect |
|---|---|
| `Cmd/TargetID = <n>` | Select which waypoint YAML to play (matched against `iiot_device.target_map`) |
| `Cmd/CntrlCmd/Start = true` | Begin playback of the selected sequence |
| `Cmd/CntrlCmd/Stop = true` | Halt playback immediately |
| `Cmd/CntrlCmd/Reset = true` | Acknowledge cycle Complete → return to Idle |
| `Cmd/CntrlCmd/Clear = true` | Clear active alarms → return to Idle from Aborted |

State is reported back via one-hot Boolean tags:

```
Status/State/Current/Idle      — true when ready for a new cycle
Status/State/Current/Execute   — true while playing a sequence
Status/State/Current/Complete  — true when sequence finished cleanly
Status/State/Current/Aborted   — true on fault (see alarm tags)
```

For the full SCADA integration contract see [docs/simple_implementation.md](#documentation).

---

## Documentation

### Configuration

| File | Contents |
|---|---|
| [config/gui_params.yaml](config/gui_params.yaml) | All runtime defaults — broker, ISA-95 identity, deadbands, CAN interface, target map |
| [config/gui_secrets.example.yaml](config/gui_secrets.example.yaml) | Template for broker credentials (copy → `gui_secrets.yaml`, gitignored) |

### Design and integration docs

| File | Audience | Contents |
|---|---|---|
| [docs/simple_implementation.md](docs/simple_implementation.md) | SCADA operator / integrator | Step-by-step operating guide — pre-flight, PackML cycle, alarm recovery, worked example |
| [docs/scada_integration.md](docs/scada_integration.md) | SCADA / MES engineer | Full Sparkplug B data model spec — all metrics, alarm catalogue, deadbands, handshake sequences for the whole packaging line |
| [docs/spb_node_spec.md](docs/spb_node_spec.md) | Developer | Sparkplug B bridge node specification — metric layout, state machine, RBE rules, topic contract |
| [docs/iiot_node_design.md](docs/iiot_node_design.md) | Developer | IIoT device mode design — WaypointManager feature requirements and ROS topic protocol |
| [docs/gui_design.md](docs/gui_design.md) | Developer | GUI architecture — panel layout, process lifecycle, Qt/rclpy threading model |
| [docs/migration_to_isa.md](docs/migration_to_isa.md) | Developer | ISA-95 migration spec — GID/Node/Device remapping, metric prefix, Boolean tag decisions |
| [docs/mqtt_spb_link.md](docs/mqtt_spb_link.md) | Developer | MQTT/Sparkplug B concept notes — UNS layout, ISA-95 vs SpB topology, southbound command routing |

---

## Topic protocol (bridge ↔ WaypointManager)

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `iiot/execute` | `std_msgs/Int32` | bridge → manager | Target ID to play |
| `iiot/halt` | `std_msgs/Empty` | bridge → manager | Cancel current playback |
| `iiot/status` | `std_msgs/String` | manager → bridge | `ready` / `started:N` / `progress:i/t` / `complete:N` / `aborted:N` / `error:msg` |

Debug a stuck handshake without touching the broker:

```bash
ros2 topic echo iiot/status
```

---

## Bench-testing the bridge without Ignition

```bash
# Terminal A — start broker if not running
mosquitto -v

# Terminal B — launch the GUI, save a sequence, enable IIoT mode

# Terminal C — simulate a SCADA Start command directly on ROS
ros2 topic pub --once /iiot/execute std_msgs/Int32 "data: 1"
# Watch:
ros2 topic echo iiot/status
```

Alternatively, publish a real Sparkplug B DCMD with `mosquitto_pub` and the Tahu CLI:

```bash
# Confirm the bridge is subscribed
mosquitto_sub -t "spBv1.0/DMATDTS/DCMD/DLSU/LS" -v
```

---

## Files

```
agx_arm_gui/
├── config/
│   ├── gui_params.yaml           # Main config — edit for your setup
│   └── gui_secrets.example.yaml  # Credentials template (copy, fill, gitignore)
├── docs/
│   ├── simple_implementation.md  # SCADA operator's guide (start here)
│   ├── scada_integration.md      # Full Sparkplug B / ISA-95 data model spec
│   ├── spb_node_spec.md          # Bridge node specification
│   ├── iiot_node_design.md       # IIoT device mode design
│   ├── gui_design.md             # GUI architecture
│   ├── migration_to_isa.md       # ISA-95 migration spec
│   └── mqtt_spb_link.md          # MQTT/SpB concept notes
├── agx_arm_gui/
│   ├── main.py                   # MainWindow + panel composition
│   ├── config_loader.py          # Typed view of gui_params.yaml + gui_secrets.yaml
│   ├── process_manager.py        # Subprocess lifecycle (CAN / arm / MoveIt / bridge)
│   ├── ros_monitor.py            # Polls `ros2 node list`
│   ├── waypoint_manager.py       # rclpy node — record / save / load / play / IIoT
│   ├── waypoint_panel.py         # Qt panel for waypoints + IIoT toggle
│   └── spb_bridge_node.py        # Sparkplug B bridge — RBE, PackML state, alarms
└── scripts/                      # Entry-point shims (installed by colcon)
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| **Arm doesn't move after Play** | Click **Enable** next to the Arm Controller row. Without it the controller silently drops every command. |
| **Play with MoveIt: arm doesn't move** | Wait until `ros2 action list` shows `/arm_controller/follow_joint_trajectory`, then retry. |
| **"no way to move the arm" in log** | Neither transport is up. Launch and Enable the Arm Controller (or launch MoveIt first). |
| **SPB bridge keeps reconnecting** | Another `spb_bridge_node` process is running with the same client ID. Find it: `pgrep -af spb_bridge_node` and kill it. |
| **`iiot/status` reports `error:no_sequence`** | `TargetID` not in `iiot_device.target_map`, or the YAML file is missing. |
| **`PrimaryHostOffline` alarm on launch** | Bridge reached the broker but never saw `STATE/IgnitionPrimary = ONLINE`. Verify `primary_host_id` matches your Ignition gateway name and that the Primary Host is enabled in Ignition. |
| **"GripperStatus unavailable" in log** | The `agx_arm_msgs` overlay is not sourced — gripper width won't be recorded but the rest works. Run `source ~/agx_arm_ws/install/setup.bash`. |
| **`tahu` import error on bridge start** | `pip3 install tahu` was not done at system level. Re-run the pip step in §Step 2 and ensure you used `--break-system-packages`. |
| **`paho.mqtt` version error** | Requires `paho-mqtt>=2.0`. Run `pip3 install --upgrade --break-system-packages paho-mqtt`. |
| **CAN interface not found** | Check `ip link show | grep can`. The USB-CAN adapter may need `sudo modprobe gs_usb`. |

---

## Kill everything the GUI launched

If the GUI crashes before stopping its panels, child processes can survive as orphans.

**Graceful shutdown:**

```bash
pkill -TERM -f 'spb_bridge_node'
pkill -TERM -f 'start_single_agx_arm.launch.py'
pkill -TERM -f 'start_single_agx_arm_moveit.launch.py'
sleep 2
```

**Force-kill survivors:**

```bash
pkill -KILL -f 'spb_bridge_node|start_single_agx_arm|agx_arm_moveit|agx_arm_ctrl_single_node'
pkill -KILL -f 'robot_state_publisher|move_group|rviz2|controller_manager|joint_state_broadcaster|ros2_control_node'
```

**Verify nothing remains:**

```bash
pgrep -af 'spb_bridge|agx_arm|move_group|rviz2|robot_state_publisher|ros2'
```

**One-liner nuke:**

```bash
pkill -KILL -f 'spb_bridge|agx_arm|ros2 (run|launch)|move_group|rviz2|robot_state_publisher|controller_manager|ros2_control_node'
```

> Avoid `killall ros2` or `pkill python3` — those hit unrelated processes.
