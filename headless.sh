#!/bin/bash
# Headless startup for the AGX Arm.
# Launches the arm controller, MoveIt2, Sparkplug B bridge, and WaypointManager
# without the GUI. Broker credentials and waypoint paths are read from
# config/gui_params.yaml + config/gui_secrets.yaml automatically.
#
# Prerequisites:
#   - udev CAN rule installed (see README Step 2) — can0 comes up on plug-in
#   - ROS 2 Humble installed at /opt/ros/humble
#   - Workspace built:  colcon build --packages-select agx_arm_gui
#
# Usage:
#   bash headless.sh
#   bash headless.sh --sim     # simulation mode (no CAN hardware required)

set -eo pipefail

# ── Config ────────────────────────────────────────────────────────────────────
WORKSPACE="${HOME}/agx_arm_ws"
CAN_IFACE="can0"
ARM_TYPE="piper"
EFFECTOR_TYPE="agx_gripper"
CAN_PORT="can0"
TCP_OFFSET="[0.0,0.0,0.0,0.0,0.0,0.0]"
SIM_MODE=false

# ── Parse args ────────────────────────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --sim) SIM_MODE=true ;;
        *) echo "Unknown argument: $arg"; exit 1 ;;
    esac
done

# ── Helpers ───────────────────────────────────────────────────────────────────
log()  { echo "[headless] $*"; }
die()  { echo "[headless] ERROR: $*" >&2; exit 1; }

PIDS=()

cleanup() {
    log "Shutting down (${#PIDS[@]} processes)..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    log "Done."
}
trap cleanup SIGINT SIGTERM EXIT

# ── Source ROS + workspace ────────────────────────────────────────────────────
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"

# ── CAN check ────────────────────────────────────────────────────────────────
if [ "$SIM_MODE" = false ]; then
    log "Checking ${CAN_IFACE}..."
    STATE=$(cat "/sys/class/net/${CAN_IFACE}/operstate" 2>/dev/null || echo "missing")
    if [ "$STATE" != "up" ]; then
        die "${CAN_IFACE} is not up (state='${STATE}'). Plug in the USB-CAN adapter."
    fi
    log "${CAN_IFACE} is up."
fi

# ── Arm controller ────────────────────────────────────────────────────────────
log "Launching arm controller..."
ARM_CMD=(
    ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py
    "arm_type:=${ARM_TYPE}"
    "effector_type:=${EFFECTOR_TYPE}"
    "tcp_offset:=${TCP_OFFSET}"
    "publish_gripper_joint:=false"
)
[ "$SIM_MODE" = false ] && ARM_CMD+=("can_port:=${CAN_PORT}")

"${ARM_CMD[@]}" &
PIDS+=($!)
log "Arm controller PID: ${PIDS[-1]}"

log "Waiting for agx_arm_ctrl_single_node..."
for i in $(seq 1 30); do
    ros2 node list 2>/dev/null | grep -q "agx_arm_ctrl_single_node" && break
    [ "$i" -eq 30 ] && die "Arm controller did not start within 30 s."
    sleep 1
done
log "Arm controller node is up."

# ── MoveIt2 ───────────────────────────────────────────────────────────────────
log "Launching MoveIt2 (no RViz)..."
MOVEIT_CMD=(
    ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py
    "arm_type:=${ARM_TYPE}"
    "effector_type:=${EFFECTOR_TYPE}"
    "follow:=true"
    "use_rviz:=false"
)
[ "$SIM_MODE" = false ] && MOVEIT_CMD+=("can_port:=${CAN_PORT}")

"${MOVEIT_CMD[@]}" &
PIDS+=($!)
log "MoveIt2 PID: ${PIDS[-1]}"

log "Waiting for /arm_controller/follow_joint_trajectory..."
for i in $(seq 1 60); do
    ros2 action list 2>/dev/null | grep -q "follow_joint_trajectory" && break
    [ "$i" -eq 60 ] && die "FollowJointTrajectory action did not appear within 60 s."
    sleep 1
done
log "Trajectory action server ready."

# ── Enable arm ────────────────────────────────────────────────────────────────
if [ "$SIM_MODE" = false ]; then
    log "Enabling arm (allowing 2 s for controllers to settle)..."
    sleep 2
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}" > /dev/null 2>&1 \
        && log "Arm enabled." \
        || log "WARNING: /enable_agx_arm call failed — arm may already be enabled."
fi

# ── Sparkplug B bridge ────────────────────────────────────────────────────────
# Broker host / credentials are read from gui_params.yaml + gui_secrets.yaml.
log "Launching Sparkplug B bridge..."
ros2 run agx_arm_gui spb_bridge_node &
PIDS+=($!)
log "SPB bridge PID: ${PIDS[-1]}"

# ── WaypointManager (IIoT device mode) ───────────────────────────────────────
# target_map is read from gui_params.yaml.
log "Launching WaypointManager (IIoT mode)..."
ros2 run agx_arm_gui waypoint_manager_node &
PIDS+=($!)
log "WaypointManager PID: ${PIDS[-1]}"

# ── Running ───────────────────────────────────────────────────────────────────
log ""
log "All services running. Press Ctrl+C to stop."
log ""
log "  SPB identity : $(python3 -c "
from agx_arm_gui.config_loader import load_config
c = load_config()
print(f'{c.spb_group_id} / {c.spb_edge_node_id} / {c.spb_device_id}')
" 2>/dev/null || echo "(config unreadable)")"
log "  Broker       : $(python3 -c "
from agx_arm_gui.config_loader import load_config
b = load_config().active_broker()
print(f'{b.host}:{b.port} (TLS={b.use_tls})')
" 2>/dev/null || echo "(config unreadable)")"

wait
