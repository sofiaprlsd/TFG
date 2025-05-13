#ifndef MAIN_NODE_HPP
#define MAIN_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "set_constraints_srv/srv/set_constraints_srv.hpp"
#include <cmath>

class MainNode : public rclcpp::Node
{
public:
    MainNode();
    void PublishCommand(double command);
    void command(std::string line);

private:
    void timer_callback();
    void call_service();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_PositionReference;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_CleanSignal;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_EffortReference;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ControlMode;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_SliderParameters;
    void subscription_SliderParameters_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    rclcpp::Client<set_constraints_srv::srv::SetConstraintsSrv>::SharedPtr client_;
    float frequency_;
    float amplitude_;
    bool is_pulse_;
    float time_;

    int mode_;
    float disturb_ampl_;
    float disturb_duration_;
    float disturb_interval_;
    float last_disturb_time_;

    float new_maxy_ = 20;
    float new_miny_ = -20;
    float new_delta_y_ = 100;
    float new_minu_ = 0.5;
    float new_maxu_ = 0.5;
    float new_delta_u_ = 0.2;
    double new_alpha_ = 0;
    int new_constraint_ = 0;
    float pos_offset_ = 0.375;
};

class LowPassFilter
{
private:
    float alpha;
    float prevOutput;

public:
    LowPassFilter(float alpha) : alpha(alpha), prevOutput(0.0f)
    {
        // Alpha debe estar entre 0 y 1 (0.1 es más suave, 0.9 es más reactivo)
    }

    float process(float input)
    {
        prevOutput = alpha * input + (1.0f - alpha) * prevOutput;
        return prevOutput;
    }

    void reset()
    {
        prevOutput = 0.0f;
    }
};

#endif