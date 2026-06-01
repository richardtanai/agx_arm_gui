> **DEPRECATED** — superseded by [spb_node_spec.md](spb_node_spec.md). Do not update this file.

1. System Context

    Edge Node (Raspberry Pi): Hardware-to-cloud gateway. Runs Sparkplug B client and AgileX CAN drivers.

    Primary Application (SCADA PC): Ignition SCADA/MQTT Engine.

    Orchestrator (Delta AS PLC): Masters motion triggers and sensor logic via Modbus.

2. Namespace & Identity

    Group ID: DMAT

    Edge Node ID: agx_arm_bridge

    Device ID: piper_arm

    Namespace: spBv1.0

3. Metric Definitions (The "Digital Twin")
A. Robot State (Status/)
Metric Name	Type	Description
Status/Overall	String	Current mode (Physical, Fake, Error, Idle).
Status/Busy	Boolean	True while a MoveIt trajectory is executing.
Status/Done	Boolean	Pulses True for 2s after successful completion.
Status/Heartbeat	Boolean	Toggles every 500ms (Watchdog).
B. Joint Telemetry (Positions/Joints/)

    Source: /joint_states (Throttled to 10Hz).

    Subscribe topic: spBv1.0/DMAT/DDATA/agx_arm_bridge/piper_arm

    Metrics: Positions/Joints/J1 through J6 (Float, Degrees).

C. End Effector Pose (Positions/EE/)

    Source: /tf (base_link → gripper_link).

    Metrics:

        Positions/EE/X, Y, Z (Meters).

        Positions/EE/Roll, Pitch, Yaw (Degrees).

4. MQTT Topics

All topics follow the Sparkplug B pattern: `spBv1.0/{group_id}/{msg_type}/{edge_node_id}[/{device_id}]`

**Published by the bridge (outbound):**

| Topic | Trigger |
|---|---|
| `spBv1.0/DMAT/NBIRTH/agx_arm_bridge` | On MQTT connect — node birth certificate with all metrics and initial values |
| `spBv1.0/DMAT/DBIRTH/agx_arm_bridge/piper_arm` | On MQTT connect — device birth certificate |
| `spBv1.0/DMAT/DDATA/agx_arm_bridge/piper_arm` | Continuously at 10 Hz — joint positions and EE pose telemetry |

**Published by the broker (Last Will Testament):**

| Topic | Trigger |
|---|---|
| `spBv1.0/DMAT/NDEATH/agx_arm_bridge` | Auto-published by broker if the bridge disconnects ungracefully (watchdog) |

**Subscribed by the bridge (inbound):**

| Topic | Description |
|---|---|
| `spBv1.0/DMAT/DCMD/agx_arm_bridge/piper_arm` | Device commands from SCADA/PLC (Trigger, TargetID, Halt) |

5. Accessing the Topics

**Terminal monitor (human-readable)**

Use the included `spb_monitor.py` script. It subscribes to DBIRTH + DDATA, decodes the protobuf payload, and prints a live updating display:

```bash
# Local broker (default)
python3 spb_monitor.py

# Remote broker with credentials
python3 spb_monitor.py --host 192.168.1.50 --port 1883 --user admin --password secret

# HiveMQ Cloud (TLS)
python3 spb_monitor.py --host <cluster>.hivemq.cloud --port 8883 --user USER --password PASS --tls
```

**Raw inspection with mosquitto_sub**

Note: payloads are binary protobuf — output will not be human-readable, but useful to confirm messages are arriving.

```bash
mosquitto_sub -h localhost -t "spBv1.0/DMAT/#" -v
```

**Python subscriber snippet**

```python
import paho.mqtt.client as mqtt
import tahu.sparkplug_b_pb2 as spb_pb2

def on_message(client, userdata, msg):
    payload = spb_pb2.Payload()
    payload.ParseFromString(msg.payload)
    for metric in payload.metrics:
        if metric.name.startswith("Positions/Joints/"):
            joint = metric.name.split("/")[-1]   # J1 … J6
            print(f"{joint}: {metric.float_value:.2f}°")

client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)
client.subscribe("spBv1.0/DMAT/DDATA/agx_arm_bridge/piper_arm", qos=0)
client.loop_forever()
```

6. Command Interface (DCMD)
Metric Name	Type	Logic
Commands/Trigger	Boolean	Starts the motion sequence.
Commands/TargetID	Int	Index for pose in config/poses.yaml.
Commands/Halt	Boolean	Immediate MoveIt cancellation.
7. Stateful Handshake Logic (4-Phase)

    Request: PLC pulses Trigger = True.

    Acknowledge: RPI sets Busy = True.

    Execution: PLC clears Trigger. RPI executes MoveIt.

    Completion: RPI sets Busy = False and Done = True.