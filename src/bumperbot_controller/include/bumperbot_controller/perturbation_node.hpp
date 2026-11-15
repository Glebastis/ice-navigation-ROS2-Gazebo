#ifndef PERTURBATION_NODE_HPP
#define PERTURBATION_NODE_HPP

#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


class PerturbationNode : public rclcpp::Node
{
public:
    PerturbationNode(const std::string& name);

private:
    void cmdCallback(const geometry_msgs::msg::Twist &msg);

    // --- Подписка / публикация ---
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // --- Параметры ---
    bool enabled_;

    // 1. Гауссов шум на команды
    double linear_noise_std_;
    double angular_noise_std_;

    // 2. Продольное проскальзывание (bias)
    double slip_ratio_;
    double slip_threshold_;

    // 3. Боковой снос через угловое возмущение
    double lateral_slip_gain_;
    double lateral_slip_frequency_;

    // 4. Кочки (пуассоновский процесс)
    double bump_rate_;
    double bump_duration_;
    double bump_magnitude_;

    // --- Состояние ---
    double bump_remaining_;
    double bump_linear_;
    double bump_angular_;
    double lateral_slip_phase_;
    rclcpp::Time last_time_;

    // --- Генераторы случайных чисел ---
    std::mt19937 rng_;
    std::normal_distribution<double> gauss_;
    std::uniform_real_distribution<double> uniform_;

    // --- Вспомогательные методы ---
    void declareAllParameters();
    void loadParameters();
};

#endif // PERTURBATION_NODE_HPP
