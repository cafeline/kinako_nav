import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from kinako_nav_bringup.scenario_loader import (
    build_default_overrides,
    load_yaml_file,
    merge_scenario_overrides,
)


def _to_bool(raw: str) -> bool:
    return str(raw).lower() in ("1", "true", "yes", "on")


def _resolve_arg(context, value):
    if isinstance(value, LaunchConfiguration):
        return context.perform_substitution(value)
    return value


def _pick_override(resolved: str, fallback: str) -> str:
    return fallback if resolved == "" else resolved


def generate_launch_description():
    pkg_bringup = get_package_share_directory("kinako_nav_bringup")

    scenario = LaunchConfiguration("scenario")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    map_hdf5_file = LaunchConfiguration("map_hdf5_file")
    regions_config_file = LaunchConfiguration("regions_config_file")
    waypoint_csv_file = LaunchConfiguration("waypoint_csv_file")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    vq_map_file = LaunchConfiguration("vq_map_file")
    auto_start = LaunchConfiguration("auto_start")

    declare_scenario = DeclareLaunchArgument("scenario", default_value="tsukuba")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_use_rviz = DeclareLaunchArgument("use_rviz", default_value="false")
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(pkg_bringup, "rviz", "navigation.rviz"),
    )
    declare_map_hdf5 = DeclareLaunchArgument("map_hdf5_file", default_value="")
    declare_regions_config = DeclareLaunchArgument("regions_config_file", default_value="")
    declare_waypoint_csv = DeclareLaunchArgument("waypoint_csv_file", default_value="")
    declare_map_yaml = DeclareLaunchArgument("map_yaml_file", default_value="")
    declare_vq_map = DeclareLaunchArgument("vq_map_file", default_value="")
    declare_auto_start = DeclareLaunchArgument("auto_start", default_value="")

    def launch_setup(context, *args, **kwargs):
        pkg_assets = get_package_share_directory("kinako_nav_assets")
        scenario_name = _resolve_arg(context, scenario)
        scenario_path = os.path.join(pkg_bringup, "config", "scenarios", f"{scenario_name}.yaml")
        scenario_params = load_yaml_file(scenario_path)
        defaults = build_default_overrides(pkg_assets, scenario_name)

        raw_auto_start = _resolve_arg(context, auto_start)
        auto_start_override = None if raw_auto_start == "" else _to_bool(raw_auto_start)

        merged_params = merge_scenario_overrides(
            scenario_params,
            {
                "map_hdf5_file": _pick_override(_resolve_arg(context, map_hdf5_file), defaults["map_hdf5_file"]),
                "regions_config_file": _pick_override(_resolve_arg(context, regions_config_file), defaults["regions_config_file"]),
                "map_yaml_file": _pick_override(_resolve_arg(context, map_yaml_file), defaults["map_yaml_file"]),
                "waypoint_csv_file": _pick_override(_resolve_arg(context, waypoint_csv_file), defaults["waypoint_csv_file"]),
                "vq_map_file": _pick_override(_resolve_arg(context, vq_map_file), defaults["vq_map_file"]),
                "use_rviz": _to_bool(_resolve_arg(context, use_rviz)),
                "auto_start": auto_start_override,
            },
        )

        tmp = tempfile.NamedTemporaryFile(mode="w", delete=False, prefix="kinako_nav_", suffix=".yaml")
        yaml.safe_dump(merged_params, tmp, allow_unicode=True)
        params_file = tmp.name
        tmp.close()

        return [
            GroupAction(
                actions=[
                    SetParameter("use_sim_time", _resolve_arg(context, use_sim_time)),
                    Node(package="pointcloud2_cutter", executable="pointcloud2_cutter_node", name="pointcloud2_cutter", parameters=[params_file], output="screen"),
                    Node(package="vq_server", executable="vq_server", name="vq_server", parameters=[params_file], output="screen", condition=IfCondition(use_rviz)),
                    Node(package="raspicat_tvvf_navigation", executable="simple_map_server", name="map_server", parameters=[params_file], output="screen"),
                    Node(package="emcl2", executable="emcl2_node", name="emcl2", parameters=[params_file], output="screen"),
                    Node(package="obstacle_tracker", executable="obstacle_tracker", name="obstacle_tracker", parameters=[params_file], output="screen"),
                    Node(package="tvvf_vo_c", executable="tvvf_vo_c_node", name="tvvf_vo_c_node", parameters=[params_file], output="screen"),
                    Node(package="velocity_smoother", executable="velocity_smoother", name="velocity_smoother", parameters=[params_file], output="screen"),
                    Node(package="raspicat_tvvf_navigation", executable="waypoint_follower_node", name="waypoint_follower_node", parameters=[params_file], output="screen"),
                    Node(package="rviz2", executable="rviz2", name="rviz2", output="screen", arguments=["-d", rviz_config_file], condition=IfCondition(use_rviz)),
                    Node(package="imu_rpy_pose", executable="imu_rpy_pose_node", name="imu_rpy_pose", parameters=[params_file], output="screen"),
                ]
            )
        ]

    ld = LaunchDescription()
    ld.add_action(declare_scenario)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_map_hdf5)
    ld.add_action(declare_regions_config)
    ld.add_action(declare_waypoint_csv)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_vq_map)
    ld.add_action(declare_auto_start)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
