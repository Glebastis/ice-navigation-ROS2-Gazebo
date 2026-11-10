#include "bumperbot_controller/noisy_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <functional>


using std::placeholders::_1;


NoisyController::NoisyController(const std::string& name)
    : Node(name)
    , wheel_radius_(0.033)
    , wheel_separation_(0.17)
    , right_wheel_prev_pos_(0.0)
    , left_wheel_prev_pos_(0.0)
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
    , encoder_noise_std_(0.005)
    , noise_generator_(std::random_device{}())
    , encoder_noise_(0.0, 0.005)
    , first_msg_(true)
{
    // --- Параметры ROS 2 ---
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);
    declare_parameter("encoder_noise_std", 0.005);

    wheel_radius_      = get_parameter("wheel_radius").as_double();
    wheel_separation_  = get_parameter("wheel_separation").as_double();
    encoder_noise_std_ = get_parameter("encoder_noise_std").as_double();

    // Пересоздаём распределение с актуальным std из параметра
    encoder_noise_ = std::normal_distribution<double>(0.0, encoder_noise_std_);

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius     " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);
    RCLCPP_INFO_STREAM(get_logger(), "Encoder noise std      " << encoder_noise_std_);

    // --- Подписка и публикация ---
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&NoisyController::jointCallback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/bumperbot_controller/odom_noisy", 10);

    // --- Инвариантные поля одометрии ---
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id  = "base_footprint_noisy";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    // --- TF ---
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id  = "base_footprint_noisy";

    prev_time_ = get_clock()->now();
}


int NoisyController::findJointIndex(const sensor_msgs::msg::JointState &msg,
                                    const std::string &name)
{
    for (size_t i = 0; i < msg.name.size(); ++i) {
        if (msg.name[i] == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}


void NoisyController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    // 1. Находим индексы колёс по имени, а не по порядку
    const int left_idx  = findJointIndex(state, "wheel_left_joint");
    const int right_idx = findJointIndex(state, "wheel_right_joint");

    if (left_idx < 0 || right_idx < 0) {
        // Сообщение не содержит нужных суставов — пропускаем
        return;
    }
    // Проверяем, что позиции для этих суставов действительно есть
    if (state.position.size() <= static_cast<size_t>(std::max(left_idx, right_idx))) {
        return;
    }

    // 2. Добавляем шум к показаниям энкодеров
    const double wheel_encoder_left  = state.position[left_idx]  + encoder_noise_(noise_generator_);
    const double wheel_encoder_right = state.position[right_idx] + encoder_noise_(noise_generator_);

    // 3. Время
    const rclcpp::Time msg_time = state.header.stamp;

    // 4. Защита от первого сообщения
    if (first_msg_) {
        left_wheel_prev_pos_  = wheel_encoder_left;
        right_wheel_prev_pos_ = wheel_encoder_right;
        prev_time_ = msg_time;
        first_msg_ = false;
        return;
    }

    const rclcpp::Duration dt = msg_time - prev_time_;
    if (dt.seconds() <= 1e-6) {
        // Дублирующееся или некорректное время — пропускаем
        return;
    }

    // 5. Приращения колёс (шум уже включён в wheel_encoder_*)
    const double dp_left  = wheel_encoder_left  - left_wheel_prev_pos_;
    const double dp_right = wheel_encoder_right - right_wheel_prev_pos_;

    // 6. Запоминаем ЗАШУМЛЁННЫЕ значения — иначе шум не накапливается!
    left_wheel_prev_pos_  = wheel_encoder_left;
    right_wheel_prev_pos_ = wheel_encoder_right;
    prev_time_ = msg_time;

    // 7. Скорости колёс (рад/с)
    const double fi_left  = dp_left  / dt.seconds();
    const double fi_right = dp_right / dt.seconds();

    // 8. Линейная и угловая скорости робота
    const double linear  = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2.0;
    const double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

    // 9. Приращение позы
    const double d_s     = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2.0;
    const double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;

    theta_ += d_theta;
    x_ += d_s * std::cos(theta_);
    y_ += d_s * std::sin(theta_);

    // 10. Собираем и публикуем одометрию
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    odom_msg_.header.stamp = msg_time;
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.twist.twist.linear.x  = linear;
    odom_msg_.twist.twist.angular.z = angular;
    odom_pub_->publish(odom_msg_);

    // 11. TF
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
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
