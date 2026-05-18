import sys
import threading

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton,
    QComboBox, QCheckBox, QPlainTextEdit,
    QDoubleSpinBox, QFrame, QSizePolicy, QLineEdit, QSpinBox,
)

from .config_loader import load_config
from .process_manager import ProcessManager, ProcessState
from .ros_monitor import RosMonitor
from .waypoint_manager import WaypointManager
from .waypoint_panel import WaypointPanel

# ROS node names to watch for in `ros2 node list`
ARM_NODE = "/agx_arm_ctrl_single_node"
MOVEIT_NODE = "/move_group"
SPB_NODE = "/spb_bridge_node"

_STATE_COLOR = {
    ProcessState.OFF:      "#888888",
    ProcessState.STARTING: "#FFAA00",
    ProcessState.ON:       "#22CC55",
    ProcessState.ERROR:    "#CC3333",
}
_STATE_LABEL = {
    ProcessState.OFF:      "OFF",
    ProcessState.STARTING: "STARTING",
    ProcessState.ON:       "ON",
    ProcessState.ERROR:    "ERROR",
}

# Stylesheet applied to the central widget in simulation mode
_SIM_STYLE = """
    QWidget#central { background-color: #1E1200; }
    QGroupBox {
        color: #FFA040;
        border: 1px solid #995500;
        border-radius: 4px;
        margin-top: 6px;
        padding-top: 6px;
    }
    QGroupBox::title { color: #FFA040; }
    QLabel  { color: #FFA040; }
    QPlainTextEdit { background-color: #120C00; color: #FFA040; }
    QPushButton { background-color: #3A2200; color: #FFA040; border: 1px solid #995500; border-radius: 3px; }
    QPushButton:hover { background-color: #553300; }
    QPushButton:disabled { color: #665522; border-color: #553300; }
    QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {
        background-color: #3A2200; color: #FFA040; border: 1px solid #995500;
    }
    QCheckBox { color: #FFA040; }
    QTableWidget {
        background-color: #120C00; color: #FFA040;
        gridline-color: #553300; border: 1px solid #995500;
    }
    QHeaderView::section {
        background-color: #3A2200; color: #FFA040; border: 1px solid #553300;
    }
    QProgressBar {
        background-color: #120C00; color: #FFA040;
        border: 1px solid #995500; border-radius: 3px; text-align: center;
    }
    QProgressBar::chunk { background-color: #995500; }
    QSlider::groove:horizontal { background: #553300; height: 6px; border-radius: 3px; }
    QSlider::handle:horizontal {
        background: #FFA040; width: 14px; margin: -5px 0; border-radius: 7px;
    }
"""


class _LogEmitter(QObject):
    """Cross-thread signal bridge for log lines."""
    line = pyqtSignal(str)


class _StatusLight(QWidget):
    """Coloured dot + text label showing a ProcessState."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        self._dot = QLabel("●")
        self._dot.setFont(QFont("monospace", 14))
        self._text = QLabel("OFF")
        layout.addWidget(self._dot)
        layout.addWidget(self._text)
        self._apply(ProcessState.OFF)

    def set_state(self, state: str):
        self._apply(state)

    def _apply(self, state: str):
        color = _STATE_COLOR[state]
        self._dot.setStyleSheet(f"color: {color};")
        self._text.setText(_STATE_LABEL[state])


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGX Arm Control")
        self.resize(1100, 980)

        self._cfg = load_config()
        self._pm = ProcessManager()
        self._waypoints = WaypointManager()
        self._arm_emitter = _LogEmitter()
        self._moveit_emitter = _LogEmitter()
        self._can_emitter = _LogEmitter()
        self._spb_emitter = _LogEmitter()
        self._ros_monitor = RosMonitor(interval=2.0)

        self._arm_emitter.line.connect(self._append_log)
        self._moveit_emitter.line.connect(self._append_log)
        self._can_emitter.line.connect(self._append_log)
        self._spb_emitter.line.connect(self._append_log)

        self._ros_monitor.start()
        self._build_ui()

        self._poll = QTimer(self)
        self._poll.timeout.connect(self._refresh_states)
        self._poll.start(500)

    # ── UI construction ───────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget(objectName="central")
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(10)
        root.setContentsMargins(12, 12, 12, 12)

        root.addLayout(self._build_header())
        root.addLayout(self._build_main_row())
        self._waypoint_panel = WaypointPanel(self._waypoints, log_fn=self._append_log)
        root.addWidget(self._waypoint_panel)
        root.addWidget(self._build_log_panel(), stretch=1)

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()

        title = QLabel("AGX Arm Control")
        title.setFont(QFont("Sans Serif", 18, QFont.Bold))
        row.addWidget(title)
        row.addStretch()

        self._sim_badge = QLabel("⚠  SIMULATION MODE")
        self._sim_badge.setStyleSheet(
            "background:#FF8800; color:white; padding:4px 14px;"
            "border-radius:4px; font-weight:bold;"
        )
        self._sim_badge.setVisible(False)
        row.addWidget(self._sim_badge)

        self._sim_check = QCheckBox("Simulated Mode")
        self._sim_check.stateChanged.connect(self._on_sim_toggled)
        row.addWidget(self._sim_check)

        return row

    def _build_main_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(self._build_can_panel(), stretch=1)
        row.addWidget(self._build_launch_panel(), stretch=2)
        return row

    def _build_can_panel(self) -> QGroupBox:
        box = QGroupBox("CAN Bus Management")
        layout = QVBoxLayout(box)
        layout.setSpacing(8)

        iface_row = QHBoxLayout()
        iface_row.addWidget(QLabel("Interface:"))
        self._can_combo = QComboBox()
        self._can_combo.addItems(["can0", "can1", "can2"])
        self._can_combo.setEditable(True)
        self._can_combo.setCurrentText(self._cfg.can_interface)
        self._can_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        iface_row.addWidget(self._can_combo)
        layout.addLayout(iface_row)

        self._can_btn = QPushButton("Activate CAN")
        self._can_btn.setMinimumHeight(34)
        self._can_btn.clicked.connect(self._on_activate_can)
        layout.addWidget(self._can_btn)

        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Link status:"))
        self._can_light = _StatusLight()
        status_row.addWidget(self._can_light)
        status_row.addStretch()
        layout.addLayout(status_row)
        layout.addStretch()

        return box

    def _build_launch_panel(self) -> QGroupBox:
        box = QGroupBox("Launch Control")
        layout = QVBoxLayout(box)
        layout.setSpacing(8)

        # ── Arm type ──────────────────────────────────────────────────
        arm_row = QHBoxLayout()
        arm_row.addWidget(QLabel("Arm:"))
        self._arm_type_combo = QComboBox()
        self._arm_type_combo.addItems(["piper", "piper_x", "piper_l", "piper_h", "nero"])
        self._arm_type_combo.setCurrentText(self._cfg.arm_type)
        arm_row.addWidget(self._arm_type_combo)
        arm_row.addStretch()
        layout.addLayout(arm_row)

        # ── Effector type ─────────────────────────────────────────────
        eff_row = QHBoxLayout()
        eff_row.addWidget(QLabel("Effector:"))
        self._effector_combo = QComboBox()
        self._effector_combo.addItems(["none", "agx_gripper", "revo2"])
        self._effector_combo.setCurrentText(self._cfg.effector_type)
        self._effector_combo.currentTextChanged.connect(self._on_effector_changed)
        eff_row.addWidget(self._effector_combo)

        self._revo2_lbl = QLabel("Side:")
        eff_row.addWidget(self._revo2_lbl)
        self._revo2_combo = QComboBox()
        self._revo2_combo.addItems(["left", "right"])
        self._revo2_combo.setCurrentText(self._cfg.revo2_side)
        eff_row.addWidget(self._revo2_combo)
        eff_row.addStretch()
        layout.addLayout(eff_row)
        self._revo2_lbl.setVisible(False)
        self._revo2_combo.setVisible(False)

        # ── TCP offset ────────────────────────────────────────────────
        tcp_row = QHBoxLayout()
        tcp_row.addWidget(QLabel("TCP Offset:"))
        self._tcp_spinboxes = []
        tcp_defaults = self._cfg.tcp_offset
        for idx, label in enumerate(("x", "y", "z", "rx", "ry", "rz")):
            tcp_row.addWidget(QLabel(label))
            sb = QDoubleSpinBox()
            sb.setRange(-10.0, 10.0)
            sb.setSingleStep(0.001)
            sb.setDecimals(3)
            sb.setValue(tcp_defaults[idx] if idx < len(tcp_defaults) else 0.0)
            sb.setFixedWidth(72)
            sb.setToolTip("meters" if label in ("x", "y", "z") else "radians")
            tcp_row.addWidget(sb)
            self._tcp_spinboxes.append(sb)
        layout.addLayout(tcp_row)

        # ── Divider ───────────────────────────────────────────────────
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        # ── Arm Controller ────────────────────────────────────────────
        arm_row = QHBoxLayout()
        self._arm_light = _StatusLight()
        arm_row.addWidget(self._arm_light)
        arm_lbl = QLabel("Arm Controller")
        arm_lbl.setMinimumWidth(120)
        arm_row.addWidget(arm_lbl)
        arm_row.addStretch()
        self._arm_enable_btn = QPushButton("Enable")
        self._arm_enable_btn.setCheckable(True)
        self._arm_enable_btn.setMinimumWidth(70)
        self._arm_enable_btn.setToolTip(
            "Calls /enable_agx_arm. The controller silently drops control/"
            "joint_states commands until enabled — required before playback."
        )
        self._arm_enable_btn.toggled.connect(self._on_arm_enable_toggled)
        self._arm_launch_btn = QPushButton("Launch")
        self._arm_launch_btn.setMinimumWidth(70)
        self._arm_launch_btn.clicked.connect(self._on_launch_arm)
        self._arm_stop_btn = QPushButton("Stop")
        self._arm_stop_btn.setMinimumWidth(60)
        self._arm_stop_btn.clicked.connect(self._on_stop_arm)
        arm_row.addWidget(self._arm_enable_btn)
        arm_row.addWidget(self._arm_launch_btn)
        arm_row.addWidget(self._arm_stop_btn)
        layout.addLayout(arm_row)

        # ── MoveIt2 ───────────────────────────────────────────────────
        moveit_row = QHBoxLayout()
        self._moveit_light = _StatusLight()
        moveit_row.addWidget(self._moveit_light)
        moveit_lbl = QLabel("MoveIt2")
        moveit_lbl.setMinimumWidth(120)
        moveit_row.addWidget(moveit_lbl)
        moveit_row.addStretch()
        self._moveit_launch_btn = QPushButton("Launch")
        self._moveit_launch_btn.setMinimumWidth(70)
        self._moveit_launch_btn.clicked.connect(self._on_launch_moveit)
        self._moveit_stop_btn = QPushButton("Stop")
        self._moveit_stop_btn.setMinimumWidth(60)
        self._moveit_stop_btn.clicked.connect(self._on_stop_moveit)
        moveit_row.addWidget(self._moveit_launch_btn)
        moveit_row.addWidget(self._moveit_stop_btn)
        layout.addLayout(moveit_row)

        # ── SPB Bridge — broker config ─────────────────────────────────
        broker_type_row = QHBoxLayout()
        broker_type_row.addWidget(QLabel("Broker:"))
        self._broker_type_combo = QComboBox()
        self._broker_type_combo.addItems(["Local MQTT", "HiveMQ Cloud"])
        self._broker_type_combo.setCurrentIndex(
            1 if self._cfg.broker_type == "hivemq" else 0
        )
        self._broker_type_combo.currentIndexChanged.connect(self._on_broker_type_changed)
        broker_type_row.addWidget(self._broker_type_combo)
        self._tls_check = QCheckBox("TLS")
        broker_type_row.addWidget(self._tls_check)
        broker_type_row.addStretch()
        layout.addLayout(broker_type_row)

        mqtt_row = QHBoxLayout()
        mqtt_row.addWidget(QLabel("Host:"))
        self._mqtt_host_edit = QLineEdit()
        self._mqtt_host_edit.setFixedWidth(180)
        self._mqtt_host_edit.setPlaceholderText("hostname or IP")
        mqtt_row.addWidget(self._mqtt_host_edit)
        mqtt_row.addWidget(QLabel("Port:"))
        self._mqtt_port_spin = QSpinBox()
        self._mqtt_port_spin.setRange(1, 65535)
        self._mqtt_port_spin.setFixedWidth(70)
        mqtt_row.addWidget(self._mqtt_port_spin)
        mqtt_row.addStretch()
        layout.addLayout(mqtt_row)

        cred_row = QHBoxLayout()
        cred_row.addWidget(QLabel("User:"))
        self._mqtt_user_edit = QLineEdit()
        self._mqtt_user_edit.setFixedWidth(110)
        self._mqtt_user_edit.setPlaceholderText("username")
        cred_row.addWidget(self._mqtt_user_edit)
        cred_row.addWidget(QLabel("Pass:"))
        self._mqtt_pass_edit = QLineEdit()
        self._mqtt_pass_edit.setFixedWidth(110)
        self._mqtt_pass_edit.setPlaceholderText("password")
        self._mqtt_pass_edit.setEchoMode(QLineEdit.Password)
        cred_row.addWidget(self._mqtt_pass_edit)
        cred_row.addStretch()
        layout.addLayout(cred_row)

        # Seed fields from config based on current broker_type selection
        self._on_broker_type_changed(self._broker_type_combo.currentIndex())

        spb_row = QHBoxLayout()
        self._spb_light = _StatusLight()
        spb_row.addWidget(self._spb_light)
        spb_lbl = QLabel("SPB Bridge (DMAT)")
        spb_lbl.setMinimumWidth(120)
        spb_row.addWidget(spb_lbl)
        spb_row.addStretch()
        self._spb_launch_btn = QPushButton("Launch")
        self._spb_launch_btn.setMinimumWidth(70)
        self._spb_launch_btn.clicked.connect(self._on_launch_spb)
        self._spb_stop_btn = QPushButton("Stop")
        self._spb_stop_btn.setMinimumWidth(60)
        self._spb_stop_btn.clicked.connect(self._on_stop_spb)
        spb_row.addWidget(self._spb_launch_btn)
        spb_row.addWidget(self._spb_stop_btn)
        layout.addLayout(spb_row)

        layout.addStretch()
        return box

    def _on_effector_changed(self, text: str):
        is_revo2 = text == "revo2"
        self._revo2_lbl.setVisible(is_revo2)
        self._revo2_combo.setVisible(is_revo2)

    def _tcp_offset_str(self) -> str:
        vals = [sb.value() for sb in self._tcp_spinboxes]
        return "[" + ", ".join(f"{v:.4f}" for v in vals) + "]"

    def _build_log_panel(self) -> QGroupBox:
        box = QGroupBox("Log Console")
        layout = QVBoxLayout(box)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setFont(QFont("Monospace", 9))
        self._log.setMinimumHeight(200)
        layout.addWidget(self._log)

        clear_btn = QPushButton("Clear")
        clear_btn.setMaximumWidth(70)
        clear_btn.clicked.connect(self._log.clear)
        layout.addWidget(clear_btn, alignment=Qt.AlignRight)

        return box

    # ── Slots ─────────────────────────────────────────────────────────────

    def _on_sim_toggled(self, state: int):
        sim = bool(state)
        self._sim_badge.setVisible(sim)
        self.centralWidget().setStyleSheet(_SIM_STYLE if sim else "")
        mode = "SIMULATION" if sim else "PHYSICAL"
        self._append_log(f"[GUI] Mode switched to {mode}")
        if sim:
            self._can_light.set_state(ProcessState.OFF)

    def _on_activate_can(self):
        interface = self._can_combo.currentText()
        if self._sim_check.isChecked():
            self._append_log(f"[CAN] Simulated mode: {interface} marked as Virtual UP")
            self._can_light.set_state(ProcessState.ON)
            return

        self._can_btn.setEnabled(False)
        self._append_log(f"[CAN] Activating {interface} ...")

        def _run():
            try:
                self._pm.activate_can(interface, self._can_emitter.line.emit)
                self._can_emitter.line.emit(f"[CAN] {interface} activated successfully")
            except Exception as exc:
                self._can_emitter.line.emit(f"[CAN] ERROR: {exc}")
            finally:
                self._can_btn.setEnabled(True)

        threading.Thread(target=_run, daemon=True).start()

    def _on_launch_arm(self):
        sim = self._sim_check.isChecked()
        can_port = self._can_combo.currentText()
        if not sim and self._pm.get_can_status(can_port) != "up":
            self._append_log(
                f"[ARM] ERROR: CAN interface '{can_port}' is not up — activate CAN first."
            )
            return
        arm_type = self._arm_type_combo.currentText()
        effector = self._effector_combo.currentText()
        tcp = self._tcp_offset_str()
        self._append_log(f"[ARM] Launching arm controller (arm={arm_type}, sim={sim}, port={can_port}, effector={effector}, tcp={tcp}) ...")
        self._pm.launch_arm_controller(sim, can_port, arm_type, effector, tcp, self._arm_emitter.line.emit)

    def _on_stop_arm(self):
        self._append_log("[ARM] Sending SIGINT to arm controller ...")
        self._pm.stop_arm_controller()
        # Stopping the controller also drops the enable state.
        self._arm_enable_btn.blockSignals(True)
        self._arm_enable_btn.setChecked(False)
        self._arm_enable_btn.setText("Enable")
        self._arm_enable_btn.blockSignals(False)

    def _on_arm_enable_toggled(self, enable: bool):
        # Visual: button text reflects intent immediately, state update
        # happens once the service responds.
        self._arm_enable_btn.setText("Enabling…" if enable else "Disabling…")
        self._arm_enable_btn.setEnabled(False)
        action = "enable" if enable else "disable"
        self._append_log(f"[ARM] Calling /enable_agx_arm({enable}) ...")

        def _done(ok: bool, msg: str):
            # Hop back to the GUI thread via the existing arm log emitter,
            # which is already wired through a queued signal connection.
            self._arm_emitter.line.emit(
                f"[ARM] enable_agx_arm({action}) → {'OK' if ok else 'FAILED'}: {msg}"
            )
            # Mutate Qt widgets via a one-shot timer so we stay on the GUI thread.
            QTimer.singleShot(0, lambda: self._finalize_enable_button(enable, ok))

        self._waypoints.enable_arm(enable, _done)

    def _finalize_enable_button(self, intended: bool, ok: bool):
        # If the call failed, snap the button back to its previous state.
        actual = intended if ok else (not intended)
        self._arm_enable_btn.blockSignals(True)
        self._arm_enable_btn.setChecked(actual)
        self._arm_enable_btn.setText("Disable" if actual else "Enable")
        self._arm_enable_btn.blockSignals(False)
        self._arm_enable_btn.setEnabled(True)

    def _on_launch_moveit(self):
        sim = self._sim_check.isChecked()
        can_port = self._can_combo.currentText()
        if not sim and self._pm.get_can_status(can_port) != "up":
            self._append_log(
                f"[MOVEIT] ERROR: CAN interface '{can_port}' is not up — activate CAN first."
            )
            return
        arm_type = self._arm_type_combo.currentText()
        effector = self._effector_combo.currentText()
        revo2_type = self._revo2_combo.currentText()
        tcp = self._tcp_offset_str()
        self._append_log(f"[MOVEIT] Launching MoveIt2 (arm={arm_type}, sim={sim}, port={can_port}, effector={effector}, tcp={tcp}) ...")
        self._pm.launch_moveit(sim, can_port, use_rviz=True,
                               arm_type=arm_type, effector_type=effector,
                               revo2_type=revo2_type, tcp_offset=tcp,
                               log=self._moveit_emitter.line.emit)

    def _on_stop_moveit(self):
        self._append_log("[MOVEIT] Sending SIGINT to MoveIt2 ...")
        self._pm.stop_moveit()

    def _on_broker_type_changed(self, index: int):
        """Pre-fill host/port/credentials and TLS checkbox when broker type changes."""
        if index == 1:  # HiveMQ Cloud
            b = self._cfg.hivemq_broker
        else:
            b = self._cfg.local_broker
        self._mqtt_host_edit.setText(b.host)
        self._mqtt_port_spin.setValue(b.port)
        self._mqtt_user_edit.setText(b.username)
        self._mqtt_pass_edit.setText(b.password)
        self._tls_check.setChecked(b.use_tls)

    def _on_launch_spb(self):
        host     = self._mqtt_host_edit.text().strip() or "localhost"
        port     = self._mqtt_port_spin.value()
        username = self._mqtt_user_edit.text().strip()
        password = self._mqtt_pass_edit.text()
        use_tls  = self._tls_check.isChecked()
        broker_label = "HiveMQ Cloud" if self._broker_type_combo.currentIndex() == 1 else "Local"
        self._append_log(
            f"[SPB] Launching Sparkplug B bridge "
            f"({broker_label} {host}:{port}, TLS={'on' if use_tls else 'off'}) ..."
        )
        self._pm.launch_spb_bridge(host, port, username, password, use_tls,
                                   self._sim_check.isChecked(),
                                   self._spb_emitter.line.emit)

    def _on_stop_spb(self):
        self._append_log("[SPB] Sending SIGINT to SPB bridge ...")
        self._pm.stop_spb_bridge()

    # ── Periodic state refresh ────────────────────────────────────────────

    def _refresh_states(self):
        nodes = self._ros_monitor.nodes

        arm_found = any(ARM_NODE in n for n in nodes)
        self._arm_light.set_state(self._pm.get_arm_state(arm_found))

        moveit_found = any(MOVEIT_NODE in n for n in nodes)
        self._moveit_light.set_state(self._pm.get_moveit_state(moveit_found))

        spb_found = any(SPB_NODE in n for n in nodes)
        self._spb_light.set_state(self._pm.get_spb_bridge_state(spb_found))

        if not self._sim_check.isChecked():
            interface = self._can_combo.currentText()
            status = self._pm.get_can_status(interface)
            self._can_light.set_state(
                ProcessState.ON if status == "up" else ProcessState.OFF
            )

    # ── Log helper ────────────────────────────────────────────────────────

    def _append_log(self, text: str):
        self._log.appendPlainText(text)
        self._log.moveCursor(QTextCursor.End)

    # ── Cleanup ───────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self._append_log("[GUI] Shutting down — sending SIGINT to all processes ...")
        self._poll.stop()
        self._ros_monitor.stop()
        self._pm.stop_all(log=self._append_log)
        try:
            self._waypoints.shutdown()
        except Exception:
            pass
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("AGX Arm Control")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
