#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class NoisyController : public rclcpp::Node
{
public:
    NoisyController(const std::string& name);

private:
    void jointCallback(const sensor_msgs::msg::JointState &msg);

    // Поиск индексов колёс в /joint_states по имени
    static int findJointIndex(const sensor_msgs::msg::JointState &msg,
                              const std::string &name);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Odometry
    double wheel_radius_;
    double wheel_separation_;
    double right_wheel_prev_pos_;
    double left_wheel_prev_pos_;
    rclcpp::Time prev_time_;
    nav_msgs::msg::Odometry odom_msg_;
    double x_;
    double y_;
    double theta_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;

    // Шум энкодеров
    double encoder_noise_std_;
    std::mt19937 noise_generator_;
    std::normal_distribution<double> encoder_noise_;

    // Защита от первого сообщения
    bool first_msg_;
};

#endif // NOISY_CONTROLLER_HPP
