"""Scene/simulator configuration for optional ROS 2 bag recording (dSPACE + Docker)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

# Match ``DSpaceSimulation._call_art_stack_reset`` (same container / workspace).
# /race_common/install is the host-mounted rebuilt workspace (see simulator.py
# comment for the 2026-04-24 rename). /opt/race_common/install is the stale baked
# one and is missing newer message types such as race_msgs/srv/SetSelectedTtl;
# ros2 bag record needs the fresh types to deserialize custom topics.
ART_STACK_DEFAULT_SETUP = "source /race_common/install/setup.bash"


def _truthy_record_ros2_bag(params: dict) -> bool:
    v = params.get("record_ros2_bag")
    if v is None:
        return False
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes", "on")
    return False


@dataclass(frozen=True)
class Ros2BagConfig:
    """Settings when recording is enabled (``record_ros2_bag`` is true)."""

    container: str
    bag_parent_dir: str
    record_all_topics: bool
    topics: tuple
    setup_source_line: str


def load_ros2_bag_config(scene: Any, sim: Any) -> Optional[Ros2BagConfig]:
    """Return config if ``record_ros2_bag`` is enabled and container is known; else ``None``."""
    params = getattr(scene, "params", None) or {}
    if not isinstance(params, dict):
        params = dict(params) if params else {}
    if not _truthy_record_ros2_bag(params):
        return None

    if params.get("ros2_bag_use_wsl") or params.get("ros2_bag_wsl_distro"):
        print(
            "[ROS2 bag] ros2_bag_use_wsl / ros2_bag_wsl_distro are ignored — "
            "using native ``docker exec`` like ART reset (``_call_art_stack_reset``)."
        )

    raw_container = params.get("ros2_bag_container")
    if raw_container is not None and str(raw_container).strip():
        container = str(raw_container).strip()
    else:
        container = str(getattr(sim, "art_stack_container", "") or "").strip()
    if not container:
        print(
            "[ROS2 bag] record_ros2_bag is True but no container: "
            "set param ros2_bag_container or simulator art_stack_container — skipping recording."
        )
        return None

    bag_parent = params.get("ros2_bag_parent_dir") or "/ros_bags"
    bag_parent = str(bag_parent).strip() or "/ros_bags"

    topics_raw = params.get("ros2_bag_topics")
    if topics_raw is None:
        record_all = True
        topics = ()
    else:
        if isinstance(topics_raw, (list, tuple)):
            topics = tuple(str(t).strip() for t in topics_raw if str(t).strip())
        else:
            topics = (str(topics_raw).strip(),) if str(topics_raw).strip() else ()
        if not topics:
            record_all = True
        else:
            record_all = False

    setup_src = params.get("ros2_bag_setup_source")
    if setup_src is None or not str(setup_src).strip():
        setup_line = ART_STACK_DEFAULT_SETUP
    else:
        setup_line = str(setup_src).strip()

    return Ros2BagConfig(
        container=container,
        bag_parent_dir=bag_parent,
        record_all_topics=record_all,
        topics=topics,
        setup_source_line=setup_line,
    )
