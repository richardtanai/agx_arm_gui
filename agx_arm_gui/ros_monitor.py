import subprocess
import threading


class RosMonitor(threading.Thread):
    """Background thread that polls `ros2 node list` on a fixed interval."""

    def __init__(self, interval: float = 2.0):
        super().__init__(daemon=True)
        self._interval = interval
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._nodes: set = set()

    @property
    def nodes(self) -> set:
        with self._lock:
            return set(self._nodes)

    def has_node(self, name: str) -> bool:
        return any(name in n for n in self.nodes)

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            try:
                result = subprocess.run(
                    ["ros2", "node", "list"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                nodes = set(result.stdout.strip().splitlines())
            except Exception:
                nodes = set()

            with self._lock:
                self._nodes = nodes

            self._stop_event.wait(self._interval)
