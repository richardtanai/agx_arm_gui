import ctypes
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable, Optional

import psutil

from .config_loader import load_config


# Linux-only: tell the kernel to send SIGTERM to a child process the moment
# its parent dies. This stops the GUI from leaking zombie ROS processes after
# a crash or `kill -9` — exactly the situation that produces multiple SPB
# bridge instances fighting over the same Sparkplug client_id.
_PR_SET_PDEATHSIG = 1


def _set_parent_death_signal():
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        libc.prctl(_PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0)
    except Exception:
        # Non-Linux or libc missing — non-fatal, we lose only the safety net.
        pass


def _kill_stragglers(
    needle: str,
    exclude_pids: set,
    log: Callable[[str], None],
    log_prefix: str = "",
) -> int:
    """Find processes whose cmdline contains `needle` and is owned by us, and
    SIGTERM them. Returns the count actually terminated.

    Used to clear orphaned ROS nodes from a previous GUI run that didn't
    shut down cleanly — multiple instances of the same Sparkplug B node will
    fight over the broker's client_id slot and produce a disconnect storm.
    """
    my_uid = os.getuid()
    killed = 0
    candidates = []
    for proc in psutil.process_iter(["pid", "uids", "cmdline"]):
        try:
            if proc.pid in exclude_pids:
                continue
            uids = proc.info.get("uids")
            if uids is None or uids.real != my_uid:
                continue
            cmdline = proc.info.get("cmdline") or []
            if not cmdline:
                continue
            joined = " ".join(cmdline)
            if needle not in joined:
                continue
            candidates.append(proc)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    for proc in candidates:
        try:
            log(f"{log_prefix} terminating straggler PID {proc.pid}: "
                f"{' '.join(proc.cmdline()[:6])} ...")
            proc.terminate()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    # Wait up to 2 s for graceful exit, then SIGKILL anything still standing.
    if candidates:
        gone, alive = psutil.wait_procs(candidates, timeout=2.0)
        killed += len(gone)
        for proc in alive:
            try:
                log(f"{log_prefix} PID {proc.pid} still alive after SIGTERM, sending SIGKILL")
                proc.kill()
                killed += 1
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
    return killed


class ProcessState:
    OFF = "OFF"
    STARTING = "STARTING"
    ON = "ON"
    ERROR = "ERROR"


class ProcessManager:
    def __init__(self):
        self._arm_proc: Optional[subprocess.Popen] = None
        self._moveit_proc: Optional[subprocess.Popen] = None
        self._spb_proc: Optional[subprocess.Popen] = None

    # ── CAN ──────────────────────────────────────────────────────────────

    def get_can_status(self, interface: str) -> str:
        path = Path(f"/sys/class/net/{interface}/operstate")
        try:
            return path.read_text().strip()
        except OSError:
            return "unknown"

    def activate_can(self, interface: str, log: Callable[[str], None]):
        can_script = Path(load_config().can_script)
        if not can_script.is_file():
            raise RuntimeError(f"CAN script not found: {can_script}")
        # Invoke through `bash` so we don't depend on the script's +x bit —
        # git and tarballs sometimes drop the executable permission, and
        # `sudo <script>` fails with the misleading "command not found".
        try:
            result = subprocess.run(
                ["sudo", "bash", str(can_script), interface],
                capture_output=True,
                text=True,
            )
            for line in result.stdout.splitlines():
                log(f"[CAN] {line}")
            for line in result.stderr.splitlines():
                log(f"[CAN] {line}")
            if result.returncode != 0:
                raise subprocess.CalledProcessError(result.returncode, can_script)
        except FileNotFoundError as exc:
            # `bash` or `sudo` itself is missing — surface the actual binary.
            raise RuntimeError(f"Required executable not found: {exc.filename or exc}")

    # ── Launch ────────────────────────────────────────────────────────────

    def launch_arm_controller(
        self,
        sim_mode: bool,
        can_port: str,
        arm_type: str,
        effector_type: str,
        tcp_offset: str,
        log: Callable[[str], None],
    ):
        cmd = ["ros2", "launch", "agx_arm_ctrl", "start_single_agx_arm.launch.py",
               f"arm_type:={arm_type}",
               f"effector_type:={effector_type}",
               f"tcp_offset:={tcp_offset}",
               "publish_gripper_joint:=false"]
        if not sim_mode:
            cmd.append(f"can_port:={can_port}")
        self._arm_proc = self._start(cmd, "[ARM]", log)

    def launch_moveit(
        self,
        sim_mode: bool,
        can_port: str,
        use_rviz: bool,
        arm_type: str,
        effector_type: str,
        revo2_type: str,
        tcp_offset: str,
        log: Callable[[str], None],
    ):
        rviz_arg = f"use_rviz:={'true' if use_rviz else 'false'}"
        common = [
            f"arm_type:={arm_type}",
            f"effector_type:={effector_type}",
            f"revo2_type:={revo2_type}",
            f"tcp_offset:={tcp_offset}",
        ]
        if sim_mode:
            cmd = ["ros2", "launch", "agx_arm_moveit", "demo.launch.py",
                   rviz_arg, *common]
        else:
            cmd = ["ros2", "launch", "agx_arm_ctrl", "start_single_agx_arm_moveit.launch.py",
                   f"can_port:={can_port}", "follow:=true", rviz_arg, *common]
        self._moveit_proc = self._start(cmd, "[MOVEIT]", log)

    def launch_spb_bridge(
        self,
        mqtt_host: str,
        mqtt_port: int,
        mqtt_username: str,
        mqtt_password: str,
        use_tls: bool,
        sim_mode: bool,
        log: Callable[[str], None],
    ):
        self.stop_spb_bridge()  # stop the instance we own first

        # Two SPB bridges with the same Sparkplug client_id will continuously
        # kick each other off the broker (looks like a disconnect/reconnect
        # storm). Sweep up any orphans from a previous GUI run before we add
        # a new one to the broker.
        killed = _kill_stragglers("spb_bridge_node", exclude_pids={os.getpid()},
                                  log=log, log_prefix="[SPB]")
        if killed:
            log(f"[SPB] Cleaned up {killed} previous spb_bridge_node instance(s); waiting 1 s for the broker to drop their sessions ...")
            # Give the broker a moment to honour the LWT/disconnect before the
            # new client arrives — otherwise the broker may briefly see two
            # sessions with the same client_id and bounce the new one too.
            time.sleep(1.0)

        cmd = [
            "ros2", "run", "agx_arm_gui", "spb_bridge_node",
            "--ros-args",
            "-p", f"mqtt_host:={mqtt_host}",
            "-p", f"mqtt_port:={mqtt_port}",
            "-p", f"use_tls:={str(use_tls).lower()}",
            "-p", f"sim_mode:={str(sim_mode).lower()}",
        ]
        if mqtt_username:
            cmd += ["-p", f"mqtt_username:={mqtt_username}",
                    "-p", f"mqtt_password:={mqtt_password}"]
        self._spb_proc = self._start(cmd, "[SPB]", log)

    # ── Stop ──────────────────────────────────────────────────────────────

    def stop_arm_controller(self):
        self._stop(self._arm_proc)
        self._arm_proc = None

    def stop_moveit(self):
        self._stop(self._moveit_proc)
        self._moveit_proc = None

    def stop_spb_bridge(self, log: Optional[Callable[[str], None]] = None):
        self._stop(self._spb_proc)
        self._spb_proc = None
        # Belt-and-braces: the SPB bridge has historically out-lived the GUI
        # (orphaned grandchild fights the next session for the broker's
        # client_id slot). Sweep any survivor whose cmdline still matches.
        if log is None:
            log = lambda _msg: None  # noqa: E731
        _kill_stragglers("spb_bridge_node", exclude_pids={os.getpid()},
                         log=log, log_prefix="[SPB]")

    def stop_all(self, log: Optional[Callable[[str], None]] = None):
        self.stop_arm_controller()
        self.stop_moveit()
        self.stop_spb_bridge(log)

    # ── State ─────────────────────────────────────────────────────────────

    def get_arm_state(self, node_found: bool) -> str:
        return self._state_of(self._arm_proc, node_found)

    def get_moveit_state(self, node_found: bool) -> str:
        return self._state_of(self._moveit_proc, node_found)

    def get_spb_bridge_state(self, node_found: bool) -> str:
        return self._state_of(self._spb_proc, node_found)

    # ── Internals ─────────────────────────────────────────────────────────

    def _start(self, cmd: list, prefix: str, log: Callable[[str], None]) -> subprocess.Popen:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            # New session so SIGINT to the GUI doesn't propagate to children;
            # PR_SET_PDEATHSIG ensures they still die when the GUI exits.
            start_new_session=True,
            preexec_fn=_set_parent_death_signal,
        )
        threading.Thread(
            target=self._stream,
            args=(proc, prefix, log),
            daemon=True,
        ).start()
        return proc

    @staticmethod
    def _stream(proc: subprocess.Popen, prefix: str, log: Callable[[str], None]):
        for line in proc.stdout:
            log(f"{prefix} {line.rstrip()}")

    @staticmethod
    def _stop(proc: Optional[subprocess.Popen]):
        """Escalate SIGINT → SIGTERM → SIGKILL so launch files have a chance
        to shut down their children cleanly before we get rough.

        Signals are sent to the whole process group (created by
        ``start_new_session=True``), not just to the immediate child. The
        actual node lives one level down — ``ros2 run`` spawns the Python
        entry-point as a grandchild, and ``ros2 launch`` spawns one process
        per node. Signalling only the parent PID leaves these grandchildren
        running and they get reparented to init."""
        if proc is None or proc.poll() is not None:
            return

        def _signal_group(sig: int) -> bool:
            try:
                pgid = os.getpgid(proc.pid)
            except ProcessLookupError:
                return False
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return False
            except PermissionError:
                # Fallback to signalling just the parent if for some reason
                # the group isn't ours. Better than silently doing nothing.
                try:
                    proc.send_signal(sig)
                except ProcessLookupError:
                    return False
            return True

        for sig, timeout in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0)):
            if not _signal_group(sig):
                return
            try:
                proc.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:
                continue

        _signal_group(signal.SIGKILL)
        try:
            proc.wait(timeout=2.0)  # reap so it doesn't become a zombie
        except (subprocess.TimeoutExpired, ProcessLookupError):
            return

    @staticmethod
    def _state_of(proc: Optional[subprocess.Popen], node_found: bool) -> str:
        if proc is None:
            return ProcessState.OFF
        ret = proc.poll()
        if ret is not None:
            return ProcessState.ERROR if ret != 0 else ProcessState.OFF
        return ProcessState.ON if node_found else ProcessState.STARTING
