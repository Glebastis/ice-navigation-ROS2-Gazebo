#include "bumperbot_controller/simple_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <functional>


using std::placeholders::_1;


SimpleController::SimpleController(const std::string& name)
    : Node(name)
    , wheel_radius_(0.033)
    , wheel_separation_(0.17)
    , right_wheel_prev_pos_(0.0)
    , left_wheel_prev_pos_(0.0)
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
    , first_msg_(true)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);
    declare_parameter("output_topic", "/cmd_vel_raw");

    wheel_radius_     = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    const std::string output_topic = get_parameter("output_topic").as_string();

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius     " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);

    // Подписка на телеоп (TwistStamped) и публикация в /cmd_vel (Twist)
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_teleop", 10,
        std::bind(&SimpleController::velCallback, this, _1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);

    // Одометрия из /joint_states
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&SimpleController::jointCallback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/bumperbot_controller/odom", 10);

    // Инвариантные поля одометрии
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id  = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    // TF
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id  = "base_footprint";

    prev_time_ = get_clock()->now();
}


int SimpleController::findJointIndex(const sensor_msgs::msg::JointState &msg,
                                     const std::string &name)
{
    for (size_t i = 0; i < msg.name.size(); ++i) {
        if (msg.name[i] == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}


void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    // Плагин diff_drive сам переводит Twist в скорости колёс.
    // Мы лишь перекладываем TwistStamped -> Twist и публикуем в /cmd_vel.
    geometry_msgs::msg::Twist cmd;
    cmd.linear = msg.twist.linear;
    cmd.angular = msg.twist.angular;
    cmd_vel_pub_->publish(cmd);
}


void SimpleController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    // 1. Индексы колёс по имени
    const int left_idx = findJointIndex(state, "wheel_left_joint");
    const int right_idx = findJointIndex(state, "wheel_right_joint");

    if (left_idx < 0 || right_idx < 0) return;
    if (state.position.size() <= static_cast<size_t>(std::max(left_idx, right_idx))) return;

    // 2. Время
    const rclcpp::Time msg_time = state.header.stamp;

    // 3. Защита от первого сообщения
    if (first_msg_) {
        left_wheel_prev_pos_ = state.position[left_idx];
        right_wheel_prev_pos_ = state.position[right_idx];
        prev_time_ = msg_time;
        first_msg_ = false;
        return;
    }

    const rclcpp::Duration dt = msg_time - prev_time_;
    if (dt.seconds() <= 1e-6) return;

    // 4. Приращения колёс (без шума — это «честная» одометрия)
    const double dp_left = state.position[left_idx]  - left_wheel_prev_pos_;
    const double dp_right = state.position[right_idx] - right_wheel_prev_pos_;

    left_wheel_prev_pos_ = state.position[left_idx];
    right_wheel_prev_pos_ = state.position[right_idx];
    prev_time_ = msg_time;

    // 5. Скорости колёс
    const double fi_left = dp_left  / dt.seconds();
    const double fi_right = dp_right / dt.seconds();

    // 6. Линейная и угловая скорости
    const double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2.0;
    const double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    // 7. Приращение позы
    const double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2.0;
    const double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * std::cos(theta_);
    y_ += d_s * std::sin(theta_);

    // 8. Публикация одометрии
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    odom_msg_.header.stamp = msg_time;
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;
    odom_pub_->publish(odom_msg_);

    // 9. TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = msg_time;
    transform_broadcaster_->sendTransform(transform_stamped_);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
