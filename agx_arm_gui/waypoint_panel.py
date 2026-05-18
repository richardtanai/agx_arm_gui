"""Waypoint Recorder & Playback GUI panel.

Self-contained QGroupBox; main.py just instantiates and adds it.
Uses a Qt signal bridge so playback callbacks (called from the rclpy executor
thread) can update the UI safely.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QDoubleSpinBox,
    QFileDialog, QMessageBox, QSlider, QProgressBar, QAbstractItemView,
    QCheckBox, QFrame,
)

from .config_loader import load_config
from .waypoint_manager import WaypointManager


class _PlaybackBridge(QObject):
    progress = pyqtSignal(int, int)        # idx, total
    finished = pyqtSignal(bool, str)       # ok, reason
    iiot_event = pyqtSignal(str)           # status string from iiot/status


class WaypointPanel(QGroupBox):
    DEFAULT_DIR = "~/agx_arm_ws/waypoints"

    def __init__(
        self,
        manager: WaypointManager,
        log_fn=None,
        parent=None,
    ):
        super().__init__("Waypoint Recorder & Playback", parent)
        self._mgr = manager
        self._log = log_fn or (lambda s: None)

        self._bridge = _PlaybackBridge()
        self._bridge.progress.connect(self._on_progress)
        self._bridge.finished.connect(self._on_finished)
        self._bridge.iiot_event.connect(self._on_iiot_event)

        self._cfg = load_config()
        self._build_ui()
        self._refresh_table()

    # ── UI ───────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(6)

        # Row 1 — record / clear / hold-time-default
        rec_row = QHBoxLayout()
        self._record_btn = QPushButton("Record Waypoint")
        self._record_btn.setMinimumHeight(32)
        self._record_btn.clicked.connect(self._on_record)
        rec_row.addWidget(self._record_btn)

        rec_row.addWidget(QLabel("Default hold:"))
        self._hold_spin = QDoubleSpinBox()
        self._hold_spin.setRange(0.0, 60.0)
        self._hold_spin.setSingleStep(0.1)
        self._hold_spin.setDecimals(2)
        self._hold_spin.setValue(1.0)
        self._hold_spin.setSuffix(" s")
        self._hold_spin.setFixedWidth(80)
        rec_row.addWidget(self._hold_spin)

        self._clear_btn = QPushButton("Clear All")
        self._clear_btn.clicked.connect(self._on_clear)
        rec_row.addWidget(self._clear_btn)

        self._delete_btn = QPushButton("Delete Selected")
        self._delete_btn.clicked.connect(self._on_delete)
        rec_row.addWidget(self._delete_btn)
        rec_row.addStretch()
        root.addLayout(rec_row)

        # Row 2 — table of waypoints
        self._table = QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(["#", "Name", "Hold (s)", "Gripper (m)"])
        self._table.verticalHeader().setVisible(False)
        self._table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.SelectedClicked)
        hdr = self._table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(1, QHeaderView.Stretch)
        hdr.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self._table.setMinimumHeight(140)
        self._table.itemChanged.connect(self._on_item_changed)
        root.addWidget(self._table)

        # Row 3 — load / save
        io_row = QHBoxLayout()
        self._load_btn = QPushButton("Load YAML…")
        self._load_btn.clicked.connect(self._on_load)
        io_row.addWidget(self._load_btn)
        self._save_btn = QPushButton("Save YAML…")
        self._save_btn.clicked.connect(self._on_save)
        io_row.addWidget(self._save_btn)
        io_row.addStretch()
        root.addLayout(io_row)

        # Row 4 — playback controls
        play_row = QHBoxLayout()
        self._play_btn = QPushButton("▶ Play")
        self._play_btn.setMinimumHeight(34)
        self._play_btn.clicked.connect(self._on_play)
        play_row.addWidget(self._play_btn)

        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        self._pause_btn.setEnabled(False)
        play_row.addWidget(self._pause_btn)

        self._stop_btn = QPushButton("Stop")
        self._stop_btn.clicked.connect(self._on_stop)
        self._stop_btn.setEnabled(False)
        play_row.addWidget(self._stop_btn)

        self._home_btn = QPushButton("Move to Home")
        self._home_btn.setToolTip(
            "Calls /move_home on the arm controller. Arm must be enabled."
        )
        self._home_btn.clicked.connect(self._on_move_home)
        play_row.addWidget(self._home_btn)

        play_row.addWidget(QLabel("Speed:"))
        self._speed_slider = QSlider(Qt.Horizontal)
        self._speed_slider.setRange(10, 200)   # 0.10x – 2.00x
        self._speed_slider.setValue(100)
        self._speed_slider.setFixedWidth(160)
        self._speed_slider.valueChanged.connect(self._on_speed_changed)
        play_row.addWidget(self._speed_slider)

        self._speed_lbl = QLabel("1.00×")
        self._speed_lbl.setFont(QFont("monospace", 10))
        self._speed_lbl.setMinimumWidth(48)
        play_row.addWidget(self._speed_lbl)
        play_row.addStretch()
        root.addLayout(play_row)

        # Row 5 — progress
        self._progress = QProgressBar()
        self._progress.setRange(0, 1)
        self._progress.setValue(0)
        self._progress.setFormat("Idle")
        root.addWidget(self._progress)

        # Divider before IIoT controls
        divider = QFrame()
        divider.setFrameShape(QFrame.HLine)
        divider.setFrameShadow(QFrame.Sunken)
        root.addWidget(divider)

        # Row 6 — IIoT Device Mode
        iiot_row = QHBoxLayout()
        self._iiot_check = QCheckBox("IIoT Device Mode")
        self._iiot_check.setToolTip(
            "When enabled, an SpB DCMD Cmd/CntrlCmd=Start plays back a recorded "
            "waypoint sequence selected by Cmd/TargetID. TargetID 0 plays the "
            "sequence currently loaded in the GUI; non-zero IDs are looked up "
            "in iiot_device.target_map (gui_params.yaml)."
        )
        self._iiot_check.toggled.connect(self._on_iiot_toggled)
        iiot_row.addWidget(self._iiot_check)

        iiot_row.addWidget(QLabel("Status:"))
        self._iiot_status_lbl = QLabel("disabled")
        self._iiot_status_lbl.setFont(QFont("monospace", 9))
        self._iiot_status_lbl.setStyleSheet("padding: 2px 6px; border: 1px solid #888; border-radius: 3px;")
        iiot_row.addWidget(self._iiot_status_lbl)
        iiot_row.addStretch()

        # Brief manual-trigger button for bench testing — publishes the same
        # iiot/execute message the SPB bridge would, against the currently-
        # selected target id.
        self._iiot_target_spin = QDoubleSpinBox()
        self._iiot_target_spin.setDecimals(0)
        self._iiot_target_spin.setRange(0, 999)
        self._iiot_target_spin.setValue(0)
        self._iiot_target_spin.setPrefix("Target: ")
        self._iiot_target_spin.setFixedWidth(110)
        iiot_row.addWidget(self._iiot_target_spin)
        root.addLayout(iiot_row)

    # ── Slots — recording / editing ──────────────────────────────────────

    def _on_record(self):
        wp = self._mgr.record_current(hold_time=self._hold_spin.value())
        if wp is None:
            QMessageBox.warning(
                self, "No data",
                "No joint state received yet — make sure the arm controller "
                "is running and feedback/joint_states is being published."
            )
            return
        self._log(f"[WP] Recorded {wp.name} ({len(wp.positions)} joints)")
        self._refresh_table()

    def _on_clear(self):
        if not self._mgr.sequence.waypoints:
            return
        if QMessageBox.question(
            self, "Clear waypoints",
            "Discard all recorded waypoints?",
            QMessageBox.Yes | QMessageBox.No,
        ) == QMessageBox.Yes:
            self._mgr.clear()
            self._refresh_table()
            self._log("[WP] Cleared all waypoints")

    def _on_delete(self):
        rows = sorted({i.row() for i in self._table.selectedIndexes()}, reverse=True)
        for r in rows:
            self._mgr.remove_at(r)
        if rows:
            self._log(f"[WP] Deleted {len(rows)} waypoint(s)")
            self._refresh_table()

    def _on_item_changed(self, item: QTableWidgetItem):
        # Only the Hold column is editable; ignore programmatic changes via the guard.
        if getattr(self, "_loading", False):
            return
        if item.column() != 2:
            return
        try:
            new_hold = float(item.text())
        except ValueError:
            self._refresh_table()
            return
        self._mgr.update_hold_time(item.row(), new_hold)

    def _refresh_table(self):
        self._loading = True
        try:
            wps = self._mgr.sequence.waypoints
            self._table.setRowCount(len(wps))
            for r, wp in enumerate(wps):
                idx_item = QTableWidgetItem(str(r + 1))
                idx_item.setFlags(idx_item.flags() & ~Qt.ItemIsEditable)
                self._table.setItem(r, 0, idx_item)

                name_item = QTableWidgetItem(wp.name)
                name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
                self._table.setItem(r, 1, name_item)

                hold_item = QTableWidgetItem(f"{wp.hold_time:.2f}")
                self._table.setItem(r, 2, hold_item)

                grip_text = f"{wp.gripper_width:.3f}" if wp.gripper_width is not None else "—"
                grip_item = QTableWidgetItem(grip_text)
                grip_item.setFlags(grip_item.flags() & ~Qt.ItemIsEditable)
                self._table.setItem(r, 3, grip_item)
        finally:
            self._loading = False
        self._progress.setRange(0, max(1, len(self._mgr.sequence.waypoints)))
        if not self._mgr.is_playing():
            self._progress.setValue(0)
            self._progress.setFormat("Idle" if not self._mgr.sequence.waypoints
                                     else f"Ready · {len(self._mgr.sequence.waypoints)} waypoint(s)")

    # ── Slots — file I/O ─────────────────────────────────────────────────

    def _default_dir(self) -> str:
        d = Path(self.DEFAULT_DIR).expanduser()
        d.mkdir(parents=True, exist_ok=True)
        return str(d)

    def _on_save(self):
        if not self._mgr.sequence.waypoints:
            QMessageBox.information(self, "Nothing to save", "Record at least one waypoint first.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save waypoint sequence", self._default_dir(), "YAML (*.yaml *.yml)"
        )
        if not path:
            return
        if not path.endswith((".yaml", ".yml")):
            path += ".yaml"
        try:
            self._mgr.save(path)
        except Exception as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        self._log(f"[WP] Saved {len(self._mgr.sequence.waypoints)} waypoint(s) → {path}")

    def _on_load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load waypoint sequence", self._default_dir(), "YAML (*.yaml *.yml)"
        )
        if not path:
            return
        try:
            self._mgr.load(path)
        except Exception as exc:
            QMessageBox.critical(self, "Load failed", str(exc))
            return
        self._refresh_table()
        self._log(f"[WP] Loaded {len(self._mgr.sequence.waypoints)} waypoint(s) from {path}")

    # ── Slots — playback ─────────────────────────────────────────────────

    def _on_play(self):
        if self._mgr.is_playing():
            return
        if not self._mgr.sequence.waypoints:
            QMessageBox.information(self, "No waypoints", "Record or load a sequence first.")
            return

        # Decide which transport will be used and surface it to the user.
        # If MoveIt is up, the trajectory action is the only thing that
        # actually moves the arm — control/joint_states is being flooded by
        # the joint_state_broadcaster.
        traj_ready = self._mgr.traj_action_is_ready()
        sub_count = self._mgr.control_subscriber_count()

        if not traj_ready and sub_count == 0:
            resp = QMessageBox.warning(
                self, "No way to move the arm",
                "Neither MoveIt's /arm_controller/follow_joint_trajectory "
                "action nor a subscriber on control/joint_states is "
                "available.\n\nLaunch the Arm Controller (and enable it) "
                "or launch MoveIt first. Continue anyway?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
            )
            if resp != QMessageBox.Yes:
                return

        speed = self._speed_slider.value() / 100.0
        self._set_playing_ui(True)
        self._mgr.play(
            speed=speed,
            progress_cb=lambda i, t: self._bridge.progress.emit(i, t),
            done_cb=lambda ok, msg: self._bridge.finished.emit(ok, msg),
        )
        transport = "FollowJointTrajectory action" if traj_ready else \
                    f"control/joint_states publish (subs={sub_count})"
        self._log(f"[WP] Playback started at {speed:.2f}× speed via {transport}")

    def _on_pause_toggled(self, paused: bool):
        self._mgr.set_paused(paused)
        self._pause_btn.setText("Resume" if paused else "Pause")
        self._log(f"[WP] Playback {'paused' if paused else 'resumed'}")

    def _on_stop(self):
        self._mgr.stop()

    def _on_move_home(self):
        self._home_btn.setEnabled(False)
        self._log("[WP] /move_home: requesting home pose ...")

        def _done(ok: bool, msg: str):
            # Hop back to the GUI thread — same trick we use elsewhere.
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(0, lambda: self._on_move_home_done(ok, msg))

        self._mgr.move_home(_done)

    def _on_move_home_done(self, ok: bool, msg: str):
        self._home_btn.setEnabled(True)
        self._log(f"[WP] /move_home → {'OK' if ok else 'FAILED'}: {msg}")
        if not ok:
            QMessageBox.warning(
                self, "Move to home failed", msg
            )

    def _on_speed_changed(self, val: int):
        speed = val / 100.0
        self._speed_lbl.setText(f"{speed:.2f}×")
        if self._mgr.is_playing():
            self._mgr.set_speed(speed)

    def _on_progress(self, idx: int, total: int):
        self._progress.setRange(0, max(1, total))
        self._progress.setValue(idx)
        self._progress.setFormat(f"Playing  {idx}/{total}")

    def _on_finished(self, ok: bool, reason: str):
        self._set_playing_ui(False)
        self._progress.setValue(self._progress.maximum() if ok else 0)
        self._progress.setFormat(reason)
        self._log(f"[WP] Playback {'finished' if ok else 'aborted'}: {reason}")

    def _set_playing_ui(self, playing: bool):
        self._record_btn.setEnabled(not playing)
        self._clear_btn.setEnabled(not playing)
        self._delete_btn.setEnabled(not playing)
        self._load_btn.setEnabled(not playing)
        self._save_btn.setEnabled(not playing)
        self._play_btn.setEnabled(not playing)
        self._pause_btn.setEnabled(playing)
        self._stop_btn.setEnabled(playing)
        if not playing:
            # Detach pause state so the next play starts unpaused.
            self._pause_btn.blockSignals(True)
            self._pause_btn.setChecked(False)
            self._pause_btn.setText("Pause")
            self._pause_btn.blockSignals(False)

    # ── IIoT Device Mode ─────────────────────────────────────────────────

    def _on_iiot_toggled(self, on: bool):
        if on:
            self._mgr.configure_iiot(
                enabled=True,
                resolve_target=self._resolve_target,
                default_speed=self._cfg.iiot_default_speed,
                event_cb=lambda text: self._bridge.iiot_event.emit(text),
            )
            self._iiot_status_lbl.setText("ready")
            self._iiot_status_lbl.setStyleSheet(
                "padding: 2px 6px; border: 1px solid #22CC55; "
                "color: #22CC55; border-radius: 3px;"
            )
            self._log("[IIOT] Device mode ENABLED — listening for SpB triggers")
        else:
            self._mgr.configure_iiot(
                enabled=False,
                resolve_target=None,
                default_speed=self._cfg.iiot_default_speed,
                event_cb=None,
            )
            self._iiot_status_lbl.setText("disabled")
            self._iiot_status_lbl.setStyleSheet(
                "padding: 2px 6px; border: 1px solid #888; "
                "color: #888; border-radius: 3px;"
            )
            self._log("[IIOT] Device mode DISABLED")

    def _resolve_target(self, target_id: int):
        # 0 → use currently-loaded sequence (handy during commissioning).
        if target_id == 0:
            seq = self._mgr.sequence
            if not seq.waypoints:
                return None
            return seq
        target_map = self._cfg.iiot_target_map
        rel = target_map.get(int(target_id))
        if not rel:
            return None
        from pathlib import Path
        p = Path(rel)
        if not p.is_absolute():
            p = Path(self._cfg.iiot_waypoints_dir) / p
        return p if p.is_file() else None

    def _on_iiot_event(self, text: str):
        self._iiot_status_lbl.setText(text)
        # Cheap colour cue without a full state machine.
        head, _, _ = text.partition(":")
        colour = {
            "ready":    "#22CC55",
            "started":  "#FFAA00",
            "progress": "#FFAA00",
            "complete": "#22CC55",
            "aborted":  "#CC8800",
            "error":    "#CC3333",
            "disabled": "#888888",
        }.get(head, "#FFA040")
        self._iiot_status_lbl.setStyleSheet(
            f"padding: 2px 6px; border: 1px solid {colour}; "
            f"color: {colour}; border-radius: 3px;"
        )
        self._log(f"[IIOT] {text}")
