Design Spec: AGX Arm Control GUI
1. Project Overview

    Workspace: ~/agx_arm_ws

    Environment: Ubuntu 22.04, ROS2 Humble

    Target Package: agx_arm_gui (Python-based)

    Purpose: A centralized dashboard to handle low-level CAN initialization, high-level MoveIt2 stack launching, and real-time process monitoring.

2. System Architecture

The GUI will act as a Process Manager. Instead of just sending ROS2 messages, it will manage the lifecycle of external bash scripts and ros2 launch commands.
Dependencies

    Backend: rclpy (ROS2 Python Client)

    Frontend: PyQt6 or PySide6 (Recommended for ROS2 compatibility)

    System: subprocess, psutil (for PID monitoring)

CRITICAL SCOPE RULE

DO NOT modify any files inside src/agx_arm_ros/. This package contains the core hardware drivers and MoveIt configuration. Claude should read these files to understand launch arguments and script locations, but all new logic must reside in src/agx_arm_gui/.

3. Functional Requirements
A. CAN Bus Management

    Target Script: ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh

    Logic:

        Dropdown menu to select interface (default: can0).

        "Activate CAN" button calls the script with sudo.

        Monitor: Check /sys/class/net/<interface>/operstate to verify if the link is up.

B. Launch Management

    Arm Controller: Execute ros2 launch agx_arm_ros arm_control.launch.py.

    MoveIt2: Execute ros2 launch agx_arm_ros moveit.launch.py.

    Requirement: The GUI must capture stdout and stderr from these processes to display in a "Log Console" area.

C. State Monitoring (The "Traffic Light" System)

The GUI buttons must reflect the actual system state:

    OFF (Gray): Process is not running.

    STARTING (Yellow): Command sent, waiting for ROS node to appear.

    ON (Green): Node found in ros2 node list.

    ERROR (Red): Process exited with non-zero code or script failed.

D. Fake Hardware / Simulation Mode

    Purpose: Allow the GUI to operate without a CAN connection or physical robot.

    Toggle: Add a "Simulated Mode" checkbox/switch in the GUI header.

    Logic Changes:

        CAN Activation: If "Simulated Mode" is ON, the "Activate CAN" step is bypassed and marked as "Virtual UP."

        Launch Arguments: When launching arm_control.launch.py or moveit.launch.py, append the argument use_fake_hardware:=true.

        Visual Feedback: The GUI background or a status badge should clearly indicate "SIMULATION MODE" to prevent accidental hardware commands.

E. Modified Execution Logic

    Condition: if (sim_mode_enabled)

        Skip subprocess.run(can_script)

        Launch command: ros2 launch agx_arm_ros moveit.launch.py use_fake_hardware:=true

    Condition: else (Physical Mode)

        Enforce CAN activation check.

        Launch command: ros2 launch agx_arm_ros moveit.launch.py use_fake_hardware:=false can_interface:=can0

4. Proposed File Structure
Plaintext

agx_arm_gui/
├── package.xml
├── setup.py
├── resource/
│   └── main_window.ui       # Optional: Qt Designer file
└── agx_arm_gui/
    ├── __init__.py
    ├── main.py              # Entry point
    ├── process_manager.py   # Logic for Popen and CAN scripts
    └── ros_monitor.py       # Threaded worker to check ROS graph


5. Execution Logic (For Claude's Reference)

    CAN Activation: Use subprocess.run(["sudo", script_path, interface], check=True).

    Launch Nodes: Use subprocess.Popen to keep the process handle so we can terminate it later.

    Graceful Shutdown: On GUI close, send SIGINT (Ctrl+C equivalent) to all running ROS launches to ensure the robot safely stops.

6. Target Launch Arguments

    can_interface (string): Passed to the controller launch file.

    use_rviz (boolean): Toggle for the MoveIt launch.