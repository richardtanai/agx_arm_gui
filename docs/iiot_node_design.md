> **DEPRECATED** — superseded by [spb_node_spec.md](spb_node_spec.md). Do not update this file.

I need the following features in the agx_arm_gui

How to record waypoints so that I could replay this later
- for joints and gripper position
- the recorded position will be stored in a yaml file

A playback function for the movement
- the yaml file contain the joint states and the 
- load the yaml file
- perform playback on the position while the speed is controlled

The robot will be a IIOT device
- the robot will be controlled by an ignition scada in thru the sparkplugb protocol



This robot will be a device on the node. and it would report ot the SCADA using the sparkplug b protocol

1. Governance Standards

Explicitly state that the system architecture follows these international pillars:

    Communication Protocol: MQTT Sparkplug 3.0. Ensures state management (NBIRTH/NDEATH) and binary efficiency via Protobuf.

    Data Modeling Hierarchy: ISA-95 Level 2/3. Organizes the namespace as Enterprise/Site/Area/Line/Station.

    Process State Machine: OMAC PackML (ISA-TR88). Standardizes the machine states (Idle, Execute, Complete, Aborted).

    HMI Design: ISA-101. High-performance, grayscale-based visualization to reduce operator fatigue.

    Alarm Management: ISA-18.2. Defines alarm rationalization and severity levels.

2. Mandatory Data Model Items (Per Station)

For every station (1–6), the design doc must specify these mandatory tags to satisfy the "Standard Handshake":

    Station_Ready (Bool): Permissive bit; must be TRUE for the sequence to start.

    Execute_Cmd (Bool): The "Trigger" from the Orchestrator (Ignition).

    Processing (Bool): Feedback from the Node (Jetson/PLC) that the task is underway.

    Job_Complete (Bool): The "Handshake Latch" indicating data is ready to be logged.

    Current_Step (Int32): Sub-state monitoring for the Digital Twin (Unity).

    Fault_Code (Int32): Standardized error reporting.

3. Connectivity Compliance (The Sparkplug B Contract)

Define how the system handles hardware and network failures:

    Last Will & Testament (LWT): Every node must define an NDEATH certificate at connection.

    Primary Host Monitoring: The Jetson Nano must monitor the STATE/Ignition topic. If the SCADA goes offline, the Robot must enter a "Safe State."

    Birth Metadata: DBIRTH payloads must include Metric Properties (PropertySets) for translating Fault_Code and Current_Step integers into human-readable strings.

4. Operational Logic Requirements

Document these functional rules to prove "Industrial Grade" logic:

    First-Out Alarm Logic: The system must identify the root cause station in a serial failure.

    Report-by-Exception (RBE): Nodes only publish data when a value changes or a heartbeat timer expires (Deadband management).

    Heartbeat/Watchdog: A toggling bit between the PLC and SCADA to verify the physical network integrity.

    Transactional Integrity: Handshakes must only be "Cleared" (Reset to 0) after the SCADA confirms successful SQL database insertion.

5. Security & Safety

    Hardware Interlock: Software commands (MQTT) cannot override physical Safety E-Stops or Light Curtains (hardwired to the PLC).

    Namespace Security: Access control levels (ACLs) on the HiveMQ broker to prevent Station 1 from writing to Station 6’s command tags.
    
 
 
  