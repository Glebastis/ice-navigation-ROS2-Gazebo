from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_controllers(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_noisy_controller = LaunchConfiguration("use_noisy_controller")

    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    encoder_noise_std = float(LaunchConfiguration("encoder_noise_std").perform(context))

    nodes = []

    # --- Резервный контур управления: simple_controller ---
    # Читает /cmd_vel_teleop (Twist), публикует /cmd_vel (Twist),
    # считает одометрию из /joint_states -> /bumperbot_controller/odom + TF odom->base_footprint
    simple_controller = Node(
        package="bumperbot_controller",
        executable="simple_controller",
        parameters=[
            {"wheel_radius": wheel_radius,
             "wheel_separation": wheel_separation,
             "use_sim_time": use_sim_time}],
        condition=IfCondition(use_simple_controller),
    )

    # --- Шумовая одометрия: noisy_controller ---
    # Читает /joint_states, публикует /bumperbot_controller/odom_noisy + TF odom->base_footprint_noisy
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
    nodes.append(simple_controller)
    nodes.append(noisy_controller)
    
    return nodes


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="True",
        description="Use /clock from Gazebo")

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller", default_value="True",
        description="Run simple_controller (reserve control loop + odom)")

    use_noisy_controller_arg = DeclareLaunchArgument(
        "use_noisy_controller", default_value="True",
        description="Run noisy_controller (simulated bad odometry)")

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", default_value="0.033")

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation", default_value="0.1402")

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error", default_value="0.005",
        description="Systematic error added to wheel radius in noisy_controller")

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error", default_value="0.02",
        description="Systematic error added to wheel separation in noisy_controller")

    encoder_noise_std_arg = DeclareLaunchArgument(
        "encoder_noise_std", default_value="0.005",
        description="Std dev of gaussian noise added to encoder readings [rad]")

    return LaunchDescription([
        use_sim_time_arg,
        use_simple_controller_arg,
        use_noisy_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        encoder_noise_std_arg,
        OpaqueFunction(function=launch_controllers),
    ])
