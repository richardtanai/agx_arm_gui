# agx_arm_gui

A PyQt5 dashboard for the Agilex Piper arm. Manages CAN, the arm controller, MoveIt2, and a Sparkplug B MQTT bridge — and lets you record/replay waypoint sequences either manually or as an **IIoT device** triggered by a SCADA primary host (Ignition).

---

## What it does

| Panel | Purpose |
|---|---|
| **CAN Bus Management** | Brings up the CAN interface via `can_activate.sh` and watches `/sys/class/net/<iface>/operstate`. |
| **Launch Control** | Starts/stops the arm controller, MoveIt2, and the Sparkplug B bridge. Includes arm type, effector, TCP offset, and inline broker configuration. |
| **Waypoint Recorder & Playback** | Snapshots `feedback/joint_states` + `feedback/gripper_status` into a YAML file, replays it through the `FollowJointTrajectory` action with a 0.10×–2.00× speed slider. |
| **IIoT Device Mode** | Toggle inside the Waypoint panel. When ON, the GUI plays back a sequence in response to a Sparkplug B DCMD from a SCADA host and reports state via ISA-95 Boolean tags. |
| **Log Console** | Aggregates stdout/stderr from every managed process. |
| **Simulation Mode** | Header checkbox. Marks CAN as virtual-UP and applies an amber colour theme — lets you exercise the GUI and ROS graph without physical hardware. |

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

### Step 2 — USB-CAN adapter udev rule (passwordless, plug-and-play)

The GUI activates the CAN interface by running `can_activate.sh` via `sudo`. Adding a udev rule removes the password prompt entirely and brings the interface up automatically every time the adapter is plugged in — no GUI interaction required.

This targets the **candleLight USB-CAN adapter** (`bytewerk 1d50:606f`, `gs_usb` driver). Run once after the adapter has been plugged in at least once:

```bash
sudo tee /etc/udev/rules.d/80-can0.rules << 'EOF'
# candleLight USB-CAN adapter (bytewerk 1d50:606f, gs_usb driver)
# Automatically names can0, sets 1 Mbit/s, and brings it up on plug-in.
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", \
  NAME="can0", \
  RUN+="/sbin/ip link set can0 type can bitrate 1000000", \
  RUN+="/sbin/ip link set can0 up"
EOF

sudo udevadm control --reload-rules
```

After this, plug the adapter in and verify:

```bash
ip link show can0
# Expected: flags include <NOARP,UP,LOWER_UP,ECHO>
```

> **Behaviour:** plug in → `can0` is up automatically; unplug → interface disappears cleanly. The GUI's **Activate CAN** button still works as a manual fallback.

> **Different adapter?** Run `udevadm info /sys/class/net/can0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID"` while it is plugged in to find the correct `idVendor`/`idProduct` values.

#### Manual bring-up (without the udev rule or GUI)

**Option A — use `can_activate.sh` directly:**

The script auto-detects the single CAN adapter, names it `can0`, and sets 1 Mbit/s:

```bash
sudo bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh
```

You can override the interface name, bitrate, or target USB port:

```bash
# Custom name and bitrate
sudo bash can_activate.sh can0 1000000

# Multiple adapters — specify which USB port to use
# Find the port with: sudo ethtool -i <iface> | grep bus-info
sudo bash can_activate.sh can0 1000000 1-2:1.0
```

If more than one CAN adapter is plugged in and no USB address is given, the script lists the detected ports and exits with an error.

**Option B — raw `ip link` commands:**

```bash
sudo modprobe gs_usb                              # load the driver if not auto-loaded
sudo ip link set can0 down                        # must be down before changing bitrate
sudo ip link set can0 type can bitrate 1000000    # set bitrate
sudo ip link set can0 up                          # bring up

# Verify
ip link show can0
# flags should include <NOARP,UP,LOWER_UP,ECHO>
```

To bring it back down cleanly:

```bash
sudo ip link set can0 down
```

> `RTNETLINK answers: Device or resource busy` means the interface is already UP. Run `sudo ip link set can0 down` first, then retry the bitrate and up commands.

---

### Step 3 — Python packages (pip, system-level)

> **Why system-level?** The ROS node (`spb_bridge_node`) is launched by the GUI as a subprocess that inherits the system Python, not a virtualenv. All packages must be visible to `python3` without any environment activation.

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

### Step 4 — Workspace packages

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

### Step 5 — Build

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

### Step 6 — MQTT broker

The Sparkplug B bridge requires an MQTT broker. For local development, Mosquitto is the simplest option:

```bash
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
```

For HiveMQ Cloud, fill in the host/credentials in the GUI's broker fields or in `config/gui_secrets.yaml`.

---

### Step 7 — Configuration

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
  revo2_side: left                 # left | right (only when effector_type is revo2)
  tcp_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # x y z rx ry rz (m / rad)

spb_bridge:
  group_id: DMATDTS_DLSU_LS_MiniFactory   # Sparkplug B / ISA-95 identity
  edge_node_id: agx_arm_bridge
  device_id: piper_arm
  primary_host_id: IgnitionPrimary        # Must match the Ignition gateway name

  # TF frames for end-effector pose in SCADA telemetry
  base_frame: base_link
  ee_frame: gripper_base                  # link6 | gripper_base | revo2_flange

  broker_type: local                      # local | hivemq

iiot_device:
  waypoints_dir: ~/agx_arm_ws/waypoints
  default_speed: 1.0
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

## Running headless (no GUI)

`headless.sh` launches the arm controller, MoveIt2, SPB bridge, and WaypointManager in IIoT device mode without the Qt window — useful for production deployments or SSH-only machines:

```bash
bash ~/agx_arm_ws/src/agx_arm_gui/headless.sh                    # local MQTT (default)
bash ~/agx_arm_ws/src/agx_arm_gui/headless.sh --hivemq           # HiveMQ Cloud
bash ~/agx_arm_ws/src/agx_arm_gui/headless.sh --sim              # simulation, local MQTT
bash ~/agx_arm_ws/src/agx_arm_gui/headless.sh --hivemq --sim     # HiveMQ + simulation
```

The script replicates the full GUI bring-up sequence:

1. Calls `can_activate.sh` to bring up the CAN interface (same script the GUI uses)
2. Launches the arm controller and waits for the node to appear
3. Enables the arm via `/enable_agx_arm`
4. Launches MoveIt2 (no RViz) and waits for the trajectory action server
5. Launches the SPB bridge with the resolved broker config
6. Launches `waypoint_manager_node` in IIoT device mode (target map from `gui_params.yaml`)

`--hivemq` overrides `broker_type` in `gui_params.yaml` and reads the host, port, TLS, and credentials from the `spb_bridge.hivemq` section of `gui_secrets.yaml`. Without the flag the script uses whatever `broker_type` is set to in `gui_params.yaml`.

Press Ctrl+C to stop all processes cleanly.

---

## Using the GUI

### Simulation Mode

Tick **Simulated Mode** in the header at any time. This:

- Marks the selected CAN interface as virtual-UP (skips the `can_activate.sh` call).
- Applies an amber colour theme so you can tell at a glance that hardware is not active.
- Passes `sim_mode:=true` to the arm controller and MoveIt2 launches, which skips the CAN driver.

Use simulation mode to exercise waypoint recording, playback, and IIoT logic without a physical arm.

---

### Bring-up sequence

1. **CAN Bus** row → click **Activate**. The status indicator turns green when `/sys/class/net/can0/operstate` reads `up`. If the indicator stays red, check the USB-CAN adapter is plugged in and run `ip link show can0`. (In simulation mode this step is automatic.)

2. **Arm type / Effector** — select the arm model and end-effector. For `revo2`, a **Side** dropdown appears. Edit the **TCP Offset** spinboxes if your tool has a known offset from the flange (x, y, z in metres; rx, ry, rz in radians).

3. **Arm Controller** row → click **Launch**. Wait until the log shows `Arm Controller ready`.

4. **Arm Controller** row → click **Enable**. This calls the `/enable_agx_arm` service. **The arm silently drops all commands until this is done.**

5. *(Optional)* **MoveIt2** row → click **Launch**. Required for RViz visualisation or IK-based motion. Wait for `move_group` to appear in `ros2 node list`. Playback always waits for the `FollowJointTrajectory` action server — ensure MoveIt is up before clicking Play.

6. *(Optional)* **SPB Bridge** row → select your broker type (Local MQTT or HiveMQ Cloud), fill in host / port / credentials / TLS, then click **Launch** if SCADA/IIoT integration is needed.

---

### Recording and replaying waypoints (manual)

1. Jog the arm to the first pose (hand-guide or jog service).
2. Click **Record Waypoint** — the current joint angles and gripper width are added to the table.
3. Repeat for each pose. Edit the **Hold (s)** column inline to set dwell time at each waypoint.
4. To remove individual waypoints, select rows and click **Delete Selected**. To start over, click **Clear All**.
5. Click **Save YAML…** — saves to `~/agx_arm_ws/waypoints/` by default.
6. Click **▶ Play** to replay. Use the **Speed** slider (0.10×–2.00×) to scale all hold times live.
7. Click **Pause** / **Resume** to hold mid-sequence without cancelling. Click **Stop** to cancel.
8. Click **Move to Home** to park the arm in the home pose.

**Playback transport:**

Playback always uses the `/arm_controller/follow_joint_trajectory` action server (provided by the controller stack started alongside MoveIt2). If the action server is not ready within 2 s of pressing Play, playback fails with an error message. Launch the arm controller and MoveIt2 first, and ensure the arm is enabled.

The arm joints (`joint1`–`joint6`) are sent to `arm_controller`; the gripper is dispatched in parallel to `gripper_controller` (if its action server is up) or via a single-joint message on `control/joint_states` as a fallback.

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
| `Cmd/TargetID = 0` | Use the sequence **currently loaded** in the GUI |
| `Cmd/TargetID = <n>` | Select waypoint YAML via `iiot_device.target_map` (n ≥ 1) |
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

**Bench test without Ignition:** use the **Target:** spinner next to the IIoT toggle to set a target ID, then publish a trigger directly:

```bash
ros2 topic pub --once /iiot/execute std_msgs/Int32 "data: 1"
ros2 topic echo iiot/status
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
| [docs/spb_node_spec.md](docs/spb_node_spec.md) | All | **Current authoritative spec** — ISA-95 identity, full tag tree, PackML state machine, alarm catalogue, ROS interface, configuration reference |
| [docs/gui_design.md](docs/gui_design.md) | Developer | GUI architecture — panel layout, process lifecycle, Qt/rclpy threading model |

---

## Topic protocol (bridge ↔ WaypointManager)

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `iiot/execute` | `std_msgs/Int32` | bridge → manager | Target ID to play (0 = current sequence) |
| `iiot/halt` | `std_msgs/Empty` | bridge → manager | Cancel current playback |
| `iiot/status` | `std_msgs/String` | manager → bridge | `ready` / `started:N` / `progress:i/t` / `complete:N` / `aborted:N` / `error:msg` |
| `iiot/target_waypoint` | `std_msgs/String` | manager → bridge | JSON pose of the current target waypoint (index, joint names, positions, gripper width) — mirrored to SCADA as `Positions/Target/*` metrics |

Debug a stuck handshake without touching the broker:

```bash
ros2 topic echo iiot/status
ros2 topic echo iiot/target_waypoint
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
mosquitto_sub -t "spBv1.0/DMATDTS_DLSU_LS_MiniFactory/DCMD/agx_arm_bridge/piper_arm" -v
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
├── scripts/
│   ├── agx_arm_gui               # GUI entry point
│   ├── spb_bridge_node           # Bridge node entry point
│   └── waypoint_manager_node     # Headless IIoT WaypointManager entry point
└── headless.sh                   # Headless startup script (no Qt required)
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| **Arm doesn't move after Play** | Click **Enable** next to the Arm Controller row. Without it the controller silently drops every command. |
| **Play fails: "FollowJointTrajectory action not available"** | MoveIt2 is not running or still starting up. Wait for `ros2 action list` to show `/arm_controller/follow_joint_trajectory`, then retry. |
| **"no way to move the arm" in log** | The `FollowJointTrajectory` action is not available and no subscriber is on `control/joint_states`. Launch and Enable the Arm Controller, then launch MoveIt2. |
| **SPB bridge keeps reconnecting** | Another `spb_bridge_node` process is running with the same client ID. Find it: `pgrep -af spb_bridge_node` and kill it. The GUI will sweep orphans automatically on the next bridge launch. |
| **`iiot/status` reports `error:no_sequence`** | `TargetID` not in `iiot_device.target_map`, or the YAML file is missing from `waypoints_dir`. |
| **`PrimaryHostOffline` alarm on launch** | Bridge reached the broker but never saw `STATE/IgnitionPrimary = ONLINE`. Verify `primary_host_id` matches your Ignition gateway name and that the Primary Host is enabled in Ignition. |
| **"GripperStatus unavailable" in log** | The `agx_arm_msgs` overlay is not sourced — gripper width won't be recorded but the rest works. Run `source ~/agx_arm_ws/install/setup.bash`. |
| **`tahu` import error on bridge start** | `pip3 install tahu` was not done at system level. Re-run the pip step in §Step 3 and ensure you used `--break-system-packages`. |
| **`paho.mqtt` version error** | Requires `paho-mqtt>=2.0`. Run `pip3 install --upgrade --break-system-packages paho-mqtt`. |
| **`No such file or directory: piper_with_gripper_description.xacro`** | `agx_arm_description` (or `agx_arm_moveit`) was not built or the install overlay was not sourced. Run `colcon build --packages-select agx_arm_description agx_arm_moveit` then `source install/setup.bash`. |
| **`Cannot infer SRDF from .../agx_arm_moveit` — using config/agx_arm.srdf`** | MoveIt fell back to the bundled SRDF because the installed share path didn't contain one. Usually harmless, but if MoveIt fails to plan, confirm `agx_arm_moveit` built cleanly and re-source. |
| **`Timeout waiting for arm to enable` / `Failed to get firmware version`** | The arm controller cannot talk to the hardware over CAN. Check that `can0` is UP (`ip link show can0`) and at 1 Mbit/s. If not, bring it up manually (see Step 2) before launching the arm controller. |
| **`RTNETLINK answers: Device or resource busy`** | The interface is already UP. Run `sudo ip link set can0 down` first, then retry the bitrate and up commands. |
| **CAN interface not found** | Check `ip link show \| grep can`. If the udev rule (Step 2) is in place, the interface comes up on plug-in. Otherwise run `sudo modprobe gs_usb` then `sudo bash can_activate.sh can0`. |

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
