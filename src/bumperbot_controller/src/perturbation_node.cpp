#include "bumperbot_controller/perturbation_node.hpp"

#include <cmath>
#include <functional>


using std::placeholders::_1;


PerturbationNode::PerturbationNode(const std::string& name)
    : Node(name)
    , enabled_(true)
    , linear_noise_std_(0.02)
    , angular_noise_std_(0.05)
    , slip_ratio_(0.0)
    , slip_threshold_(0.01)
    , lateral_slip_gain_(0.0)
    , lateral_slip_frequency_(1.0)
    , bump_rate_(0.0)
    , bump_duration_(0.0)
    , bump_magnitude_(0.0)
    , bump_remaining_(0.0)
    , bump_linear_(0.0)
    , bump_angular_(0.0)
    , lateral_slip_phase_(0.0)
    , rng_(std::random_device{}())
    , gauss_(0.0, 1.0)
    , uniform_(0.0, 1.0)
{
    declareAllParameters();
    loadParameters();

    last_time_ = get_clock()->now();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&PerturbationNode::cmdCallback, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_raw", 10);

    RCLCPP_INFO(get_logger(), "Perturbation node started (enabled=%s)",
                enabled_ ? "true" : "false");
}


void PerturbationNode::declareAllParameters()
{
    declare_parameter("enabled",             true);
    declare_parameter("linear_noise_std",    0.02);
    declare_parameter("angular_noise_std",   0.05);
    declare_parameter("slip_ratio",          0.0);
    declare_parameter("slip_threshold",      0.01);
    declare_parameter("lateral_slip_gain",   0.0);
    declare_parameter("lateral_slip_frequency", 1.0);
    declare_parameter("bump_rate",           0.0);
    declare_parameter("bump_duration",       0.0);
    declare_parameter("bump_magnitude",      0.0);
}


void PerturbationNode::loadParameters()
{
    enabled_                 = get_parameter("enabled").as_bool();
    linear_noise_std_        = get_parameter("linear_noise_std").as_double();
    angular_noise_std_       = get_parameter("angular_noise_std").as_double();
    slip_ratio_              = get_parameter("slip_ratio").as_double();
    slip_threshold_          = get_parameter("slip_threshold").as_double();
    lateral_slip_gain_       = get_parameter("lateral_slip_gain").as_double();
    lateral_slip_frequency_  = get_parameter("lateral_slip_frequency").as_double();
    bump_rate_               = get_parameter("bump_rate").as_double();
    bump_duration_           = get_parameter("bump_duration").as_double();
    bump_magnitude_          = get_parameter("bump_magnitude").as_double();
}


void PerturbationNode::cmdCallback(const geometry_msgs::msg::Twist &msg)
{
    // Если возмутитель выключен — пропускаем команду без изменений
    if (!enabled_) {
        cmd_pub_->publish(msg);
        return;
    }

    // Время с последнего вызова (для кочек и фазового сноса)
    const rclcpp::Time now = get_clock()->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.05;  // защита от первого вызова или скачка времени
    }
    last_time_ = now;

    geometry_msgs::msg::Twist out = msg;

    // ---------------------------------------------------------------
    // 1. Продольное проскальзывание (bias, накапливается)
    //    Работает только если робот реально движется.
    // ---------------------------------------------------------------
    if (std::abs(out.linear.x) > slip_threshold_ && slip_ratio_ > 0.0) {
        out.linear.x *= (1.0 - slip_ratio_);
    }

    // ---------------------------------------------------------------
    // 2. Гауссов шум (нулевое среднее, не накапливается)
    // ---------------------------------------------------------------
    if (linear_noise_std_ > 0.0) {
        out.linear.x += gauss_(rng_) * linear_noise_std_;
    }
    if (angular_noise_std_ > 0.0) {
        out.angular.z += gauss_(rng_) * angular_noise_std_;
    }

    // ---------------------------------------------------------------
    // 3. Боковой снос -> угловое возмущение
    //    Синусоидальный дрейф с частотой lateral_slip_frequency
    //    и амплитудой, пропорциональной текущей линейной скорости.
    //    Это имитирует "виляние" на скользкой поверхности.
    // ---------------------------------------------------------------
    if (lateral_slip_gain_ > 0.0) {
        lateral_slip_phase_ += 2.0 * M_PI * lateral_slip_frequency_ * dt;
        if (lateral_slip_phase_ > 2.0 * M_PI) {
            lateral_slip_phase_ -= 2.0 * M_PI;
        }
        const double slip_component =
            lateral_slip_gain_ * out.linear.x * std::sin(lateral_slip_phase_);
        out.angular.z += slip_component;
    }

    // ---------------------------------------------------------------
    // 4. Кочки (пуассоновский процесс + гауссова амплитуда)
    //    Редкие кратковременные импульсы.
    // ---------------------------------------------------------------
    if (bump_rate_ > 0.0 && bump_duration_ > 0.0 && bump_magnitude_ > 0.0) {
        // Если кочка уже активна — уменьшаем таймер
        if (bump_remaining_ > 0.0) {
            bump_remaining_ -= dt;
            out.linear.x  += bump_linear_;
            out.angular.z += bump_angular_;
        } else {
            // Проверяем, не началась ли новая кочка
            // Вероятность за dt: lambda * dt
            if (uniform_(rng_) < bump_rate_ * dt) {
                bump_remaining_ = bump_duration_;
                bump_linear_    = gauss_(rng_) * bump_magnitude_ * 0.5;
                bump_angular_   = gauss_(rng_) * bump_magnitude_;
                out.linear.x  += bump_linear_;
                out.angular.z += bump_angular_;
            }
        }
    }

    cmd_pub_->publish(out);
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerturbationNode>("perturbation_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
