from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("frenet_mpc"),
        "config",
        "frenet_mpc.yaml",
    ])

    params = {
        "config_file": ("", default_config, "Path to Frenet MPC YAML config."),
        "centerline_file": ("str", "/home/nvidia/f1tenth_ws/src/centerline.csv",
                            "Ordered closed-loop centerline/raceline CSV."),
        "pose_topic": ("str", "/pf/viz/inferred_pose", "Pose input topic."),
        "odom_topic": ("str", "/odom_ekf", "Odometry input topic."),
        "drive_topic": ("str", "/drive", "Drive output topic."),
        "scan_topic": ("str", "/scan", "LaserScan topic."),
        "map_topic": ("str", "/map", "OccupancyGrid topic."),
        "global_frame": ("str", "map", "TF frame for visualization."),
        "target_speed": ("num", "5.0", "Target speed (m/s)."),
        "min_speed_cmd": ("num", "0.30", "Minimum commanded speed (m/s)."),
        "max_speed": ("num", "8.0", "Maximum speed (m/s)."),
        "resample_ds": ("num", "0.05", "Centerline resample spacing (m)."),
        "v_min_corner": ("num", "2.5", "Minimum corner speed (m/s)."),
        "curvature_speed_gain": ("num", "1.8", "Curvature-to-speed exponential gain."),
        "max_search_dist_warn": ("num", "1.5", "Projection distance threshold (m)."),
        "max_steering_angle": ("num", "0.6189", "Max steering angle (rad)."),
        "max_steering_rate": ("num", "3.2", "Max steering rate (rad/s)."),
        "reverse_centerline": ("bool", "false", "Reverse centerline direction."),
        "rti_iterations": ("int", "3", "SQP-RTI iterations per solve."),
        "max_brake_decel": ("num", "5.0", "Max braking decel (m/s^2)."),
        "max_accel_forward": ("num", "4.0", "Friction circle accel budget (m/s^2)."),
        "enable_obstacle_avoidance": ("bool", "true", "Enable obstacle avoidance."),
        "obs_safety_margin": ("num", "0.10", "Lateral safety margin (m)."),
        "obs_car_half_width": ("num", "0.16", "Opponent half-width (m)."),
        "obs_car_half_length": ("num", "0.25", "Opponent half-length (m)."),
        "obs_detect_range": ("num", "6.0", "Max detection range (m)."),
        "obs_cluster_dist": ("num", "0.20", "Clustering radius (m)."),
        "obs_min_cluster_pts": ("int", "5", "Min cluster points."),
        "obs_side_commit_time": ("num", "0.5", "Side-selection hysteresis (s)."),
        "obs_map_diff_thresh": ("num", "0.20", "Wall neighborhood radius (m)."),
        "track_width_default": ("num", "1.20", "Fallback track half-width (m)."),
        "track_wall_margin": ("num", "0.18", "Wall safety margin (m)."),
        "emergency_brake_speed": ("num", "0.0", "Speed when all solves fail."),
        "w_ey": ("num", "35.0", "Lateral error weight."),
        "w_epsi": ("num", "15.0", "Heading error weight."),
        "w_v": ("num", "3.0", "Speed error weight."),
        "w_delta": ("num", "12.0", "Steering angle weight."),
        "w_delta_dot": ("num", "65.0", "Steering rate weight."),
        "w_a": ("num", "0.20", "Acceleration weight."),
        "w_ey_e": ("num", "45.0", "Terminal lateral error weight."),
        "w_epsi_e": ("num", "18.0", "Terminal heading error weight."),
        "w_v_e": ("num", "3.0", "Terminal speed error weight."),
        "w_delta_e": ("num", "1.0", "Terminal steering weight."),
    }

    launch_args = []
    node_overrides = {}

    for name, (ptype, default, desc) in params.items():
        if name == "config_file":
            launch_args.append(DeclareLaunchArgument(name, default_value=default, description=desc))
            continue
        launch_args.append(DeclareLaunchArgument(name, default_value=default, description=desc))
        node_overrides[name] = LaunchConfiguration(name)

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(launch_args + [
        Node(
            package="frenet_mpc",
            executable="frenet_mpc_node",
            name="frenet_mpc",
            output="screen",
            parameters=[config_file, node_overrides],
        ),
    ])