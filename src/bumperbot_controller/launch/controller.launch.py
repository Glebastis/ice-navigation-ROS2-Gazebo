from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os
from ament_index_python.packages import get_package_share_directory


def launch_controllers(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_noisy_controller = LaunchConfiguration("use_noisy_controller")
    use_perturbation = LaunchConfiguration("use_perturbation")

    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    encoder_noise_std = float(LaunchConfiguration("encoder_noise_std").perform(context))

    use_perturbation_bool = LaunchConfiguration("use_perturbation").perform(context).lower() == "true"

    nodes = []

    # simple_controller: если возмутитель включён — публикуем в /cmd_vel,
    # иначе сразу в /cmd_vel_raw (обход возмутителя)
    simple_output_topic = "/cmd_vel" if use_perturbation_bool else "/cmd_vel_raw"

    simple_controller = Node(
        package="bumperbot_controller",
        executable="simple_controller",
        parameters=[
            {"wheel_radius": wheel_radius,
             "wheel_separation": wheel_separation,
             "output_topic": simple_output_topic,
             "use_sim_time": use_sim_time}],
        condition=IfCondition(use_simple_controller),
    )

    noisy_controller = Node(
        package="bumperbot_controller",
        executable="noisy_controller",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "encoder_noise_std": encoder_noise_std,
             "use_sim_time": use_sim_time}],
        condition=IfCondition(use_noisy_controller),
    )

    perturbation_node = Node(
        package="bumperbot_controller",
        executable="perturbation_node",
        parameters=[
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "config", "perturbation_params.yaml"),
            {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_perturbation),
    )

    nodes.append(simple_controller)
    nodes.append(noisy_controller)
    nodes.append(perturbation_node)

    return nodes


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="True")

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller", default_value="True")

    use_noisy_controller_arg = DeclareLaunchArgument(
        "use_noisy_controller", default_value="True")

    use_perturbation_arg = DeclareLaunchArgument(
        "use_perturbation", default_value="False",
        description="Enable perturbation node (cmd_vel -> cmd_vel_raw)")

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", default_value="0.033")

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation", default_value="0.1402")

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error", default_value="0.015")

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error", default_value="0.05")

    encoder_noise_std_arg = DeclareLaunchArgument(
        "encoder_noise_std", default_value="0.02")

    return LaunchDescription([
        use_sim_time_arg,
        use_simple_controller_arg,
        use_noisy_controller_arg,
        use_perturbation_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        encoder_noise_std_arg,
        OpaqueFunction(function=launch_controllers),
    ])
