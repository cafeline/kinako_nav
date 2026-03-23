import copy
import os
from typing import Any, Dict, List

import yaml

CORE_SCENARIOS = ["tsukuba", "tsudanuma", "19f"]


def load_yaml_file(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be mapping: {path}")
    return data


def list_available_scenarios(scenarios_dir: str) -> List[str]:
    if not os.path.isdir(scenarios_dir):
        return CORE_SCENARIOS.copy()

    names = []
    for entry in os.listdir(scenarios_dir):
        if not entry.endswith(".yaml"):
            continue
        names.append(os.path.splitext(entry)[0])

    if not names:
        return CORE_SCENARIOS.copy()

    # Keep core scenarios first when present, then append others.
    ordered = [name for name in CORE_SCENARIOS if name in names]
    for name in sorted(names):
        if name not in ordered:
            ordered.append(name)
    return ordered


def build_default_overrides(assets_share_dir: str, scenario_name: str) -> Dict[str, str]:
    scenario_root = os.path.join(assets_share_dir, "maps3d", scenario_name)
    return {
        "map_hdf5_file": os.path.join(scenario_root, "localization.h5"),
        "vq_map_file": os.path.join(scenario_root, "visualization.h5"),
        "map_yaml_file": os.path.join(assets_share_dir, "maps2d", scenario_name, "navigation_map.yaml"),
        "waypoint_csv_file": os.path.join(assets_share_dir, "waypoints", scenario_name, "waypoints.csv"),
        "regions_config_file": os.path.join(assets_share_dir, "regions", scenario_name, "regions.yaml"),
    }


def _set_nested(params: Dict[str, Any], node: str, key: str, value: Any) -> None:
    params.setdefault(node, {}).setdefault("ros__parameters", {})[key] = value


def merge_scenario_overrides(
    base_params: Dict[str, Any],
    overrides: Dict[str, Any],
) -> Dict[str, Any]:
    merged = copy.deepcopy(base_params)

    mapping = {
        "map_hdf5_file": ("emcl2", "map_hdf5_path"),
        "regions_config_file": ("pointcloud2_cutter", "regions_config_path"),
        "map_yaml_file": ("map_server", "map_yaml_path"),
        "waypoint_csv_file": ("waypoint_follower_node", "waypoint_csv_path"),
        "vq_map_file": ("vq_server", "map_file"),
    }

    for override_key, (node_name, param_name) in mapping.items():
        value = overrides.get(override_key)
        if value is None:
            continue
        if isinstance(value, str) and value == "":
            continue
        _set_nested(merged, node_name, param_name, value)

    use_rviz = overrides.get("use_rviz")
    if use_rviz is not None:
        merged.setdefault("/**", {}).setdefault("ros__parameters", {})["use_rviz"] = bool(use_rviz)

    auto_start = overrides.get("auto_start")
    if auto_start is not None:
        _set_nested(merged, "waypoint_follower_node", "auto_start", bool(auto_start))

    return merged
