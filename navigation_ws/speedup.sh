#!/usr/bin/env bash
set -eu

ros2 param set /tvvf_vo_c_node costmap_min_speed 0.7
ros2 param set /tvvf_vo_c_node costmap_max_speed 0.8
ros2 param set /tvvf_vo_c_node max_linear_velocity 0.8
ros2 param set /velocity_smoother min_linear_speed 0.7
ros2 param set /velocity_smoother max_linear_speed 0.8

ros2 service call /resume_waypoint_navigation std_srvs/srv/Trigger
