#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "std_msgs/msg/float32.hpp"
#include "main/motorControl.h"
#include "set_constraints_srv/srv/set_constraints_srv.hpp"

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();
    std::shared_ptr<hebi::Group> group_;
    float position_;
    float effort_;
    bool is_receiving_ = false;
    int mode_;
    float pos_0_;

private:
    void timer_callback();
    void call_service();
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_PositionReference_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_EffortReference_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_ControlMode_;
    rclcpp::Service<set_constraints_srv::srv::SetConstraintsSrv>::SharedPtr service_setConstraints_;
    rclcpp::TimerBase::SharedPtr timer_;
    void subscription_PositionReference_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void subscription_EffortReference_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void subscription_ControlMode_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void handle_service_setConstraints(
    const std::shared_ptr<set_constraints_srv::srv::SetConstraintsSrv::Request> request,
    std::shared_ptr<set_constraints_srv::srv::SetConstraintsSrv::Response> response);
    float time_;
};

#endif