"""Load gui_params.yaml, resolving the file via (in priority order):
1. AGX_ARM_CONFIG environment variable
2. ament share directory  (installed package)
3. Sibling config/ directory  (in-source / colcon dev install)
"""

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_DEFAULT_FILENAME = "gui_params.yaml"
_SECRETS_FILENAME = "gui_secrets.yaml"


def _locate_config() -> Path:
    env_override = os.environ.get("AGX_ARM_CONFIG")
    if env_override:
        p = Path(env_override).expanduser()
        if p.is_file():
            return p
        raise FileNotFoundError(f"AGX_ARM_CONFIG points to missing file: {p}")

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("agx_arm_gui"))
        candidate = share / "config" / _DEFAULT_FILENAME
        if candidate.is_file():
            return candidate
    except Exception:
        pass

    # In-source fallback: config/ lives next to the agx_arm_gui/ sub-package
    here = Path(__file__).parent
    candidate = here.parent / "config" / _DEFAULT_FILENAME
    if candidate.is_file():
        return candidate

    raise FileNotFoundError(
        f"Could not locate {_DEFAULT_FILENAME}. "
        "Set AGX_ARM_CONFIG or rebuild/install the package."
    )


def _locate_secrets(config_path: Path) -> Path | None:
    """Find gui_secrets.yaml. Returns None if absent (allowed)."""
    env_override = os.environ.get("AGX_ARM_SECRETS")
    if env_override:
        p = Path(env_override).expanduser()
        if p.is_file():
            return p
        raise FileNotFoundError(f"AGX_ARM_SECRETS points to missing file: {p}")

    candidate = config_path.parent / _SECRETS_FILENAME
    return candidate if candidate.is_file() else None


def _deep_merge(base: dict, overlay: dict) -> dict:
    """Recursively merge overlay into base; overlay wins on scalar conflicts."""
    for k, v in overlay.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def _deep_get(d: dict, *keys, default=None) -> Any:
    for k in keys:
        if not isinstance(d, dict):
            return default
        d = d.get(k, {})
    return d if d != {} else default


@dataclass
class BrokerConfig:
    host: str
    port: int
    use_tls: bool
    username: str
    password: str


class AppConfig:
    """Typed view over gui_params.yaml."""

    def __init__(self, data: dict):
        self._d = data

    # ── CAN ──────────────────────────────────────────────────────────────

    @property
    def can_interface(self) -> str:
        return _deep_get(self._d, "can", "interface", default="can0")

    @property
    def can_script(self) -> str:
        raw = _deep_get(self._d, "can", "script",
                        default="~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh")
        return str(Path(raw).expanduser())

    # ── Arm controller ────────────────────────────────────────────────────

    @property
    def arm_type(self) -> str:
        return _deep_get(self._d, "arm_controller", "arm_type", default="piper")

    @property
    def effector_type(self) -> str:
        return _deep_get(self._d, "arm_controller", "effector_type", default="none")

    @property
    def revo2_side(self) -> str:
        return _deep_get(self._d, "arm_controller", "revo2_side", default="left")

    @property
    def tcp_offset(self) -> list:
        val = _deep_get(self._d, "arm_controller", "tcp_offset",
                        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        return list(val)

    # ── Sparkplug B bridge — identity ─────────────────────────────────────

    @property
    def spb_group_id(self) -> str:
        return _deep_get(self._d, "spb_bridge", "group_id", default="DMAT")

    @property
    def spb_edge_node_id(self) -> str:
        return _deep_get(self._d, "spb_bridge", "edge_node_id", default="agx_arm_bridge")

    @property
    def spb_device_id(self) -> str:
        return _deep_get(self._d, "spb_bridge", "device_id", default="piper_arm")

    @property
    def base_frame(self) -> str:
        return _deep_get(self._d, "spb_bridge", "base_frame", default="base_link")

    @property
    def ee_frame(self) -> str:
        return _deep_get(self._d, "spb_bridge", "ee_frame", default="link6")

    @property
    def primary_host_id(self) -> str:
        return _deep_get(self._d, "spb_bridge", "primary_host_id", default="IgnitionPrimary")

    @property
    def joint_deadband_deg(self) -> float:
        return float(_deep_get(self._d, "spb_bridge", "joint_deadband_deg", default=0.1))

    @property
    def ee_deadband_m(self) -> float:
        return float(_deep_get(self._d, "spb_bridge", "ee_deadband_m", default=0.001))

    @property
    def ee_deadband_deg(self) -> float:
        return float(_deep_get(self._d, "spb_bridge", "ee_deadband_deg", default=0.5))

    @property
    def gripper_max_width_m(self) -> float:
        # Stroke of the agx_gripper is 0.1 m; override here for other grippers.
        return float(_deep_get(self._d, "spb_bridge", "gripper_max_width_m", default=0.1))

    @property
    def gripper_deadband(self) -> float:
        # Fraction of full stroke. 0.01 = 1 % of opening.
        return float(_deep_get(self._d, "spb_bridge", "gripper_deadband", default=0.01))


    # ── Sparkplug B bridge — broker selection ─────────────────────────────

    @property
    def broker_type(self) -> str:
        """Returns 'local' or 'hivemq'."""
        return _deep_get(self._d, "spb_bridge", "broker_type", default="local")

    @property
    def local_broker(self) -> BrokerConfig:
        s = _deep_get(self._d, "spb_bridge", "local") or {}
        return BrokerConfig(
            host=s.get("host", "localhost"),
            port=int(s.get("port", 1883)),
            use_tls=bool(s.get("use_tls", False)),
            username=s.get("username", "") or "",
            password=s.get("password", "") or "",
        )

    @property
    def hivemq_broker(self) -> BrokerConfig:
        s = _deep_get(self._d, "spb_bridge", "hivemq") or {}
        return BrokerConfig(
            host=s.get("host", ""),
            port=int(s.get("port", 8883)),
            use_tls=bool(s.get("use_tls", True)),
            username=s.get("username", "") or "",
            password=s.get("password", "") or "",
        )

    def active_broker(self) -> BrokerConfig:
        """Return the broker config selected by broker_type."""
        if self.broker_type == "hivemq":
            return self.hivemq_broker
        return self.local_broker

    # ── IIoT Device Mode ─────────────────────────────────────────────────

    @property
    def iiot_waypoints_dir(self) -> str:
        raw = _deep_get(self._d, "iiot_device", "waypoints_dir",
                        default="~/agx_arm_ws/waypoints")
        return str(Path(raw).expanduser())

    @property
    def iiot_default_speed(self) -> float:
        return float(_deep_get(self._d, "iiot_device", "default_speed", default=1.0))

    @property
    def iiot_target_map(self) -> dict:
        raw = _deep_get(self._d, "iiot_device", "target_map", default={}) or {}
        # YAML keys may parse as ints or strings — normalise to int.
        out = {}
        for k, v in raw.items():
            try:
                out[int(k)] = str(v)
            except (TypeError, ValueError):
                continue
        return out


_cached: AppConfig | None = None


def load_config(reload: bool = False) -> AppConfig:
    global _cached
    if _cached is None or reload:
        path = _locate_config()
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        secrets_path = _locate_secrets(path)
        if secrets_path is not None:
            with open(secrets_path) as f:
                secrets = yaml.safe_load(f) or {}
            _deep_merge(data, secrets)
        _cached = AppConfig(data)
    return _cached
