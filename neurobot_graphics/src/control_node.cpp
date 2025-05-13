#include "main/control_node.hpp"

using namespace hebi;

ControlNode::ControlNode()
    : Node("control_node"), group_(nullptr), position_(0.0), effort_(0.0), mode_(1), pos_0_(0.0), time_(0.0)
{
    subscription_PositionReference_ = this->create_subscription<std_msgs::msg::Float32>(
        "PositionReference", 10,
        std::bind(&ControlNode::subscription_PositionReference_callback, this, std::placeholders::_1));
    subscription_EffortReference_ = this->create_subscription<std_msgs::msg::Float32>(
        "EffortReference", 10,
        std::bind(&ControlNode::subscription_EffortReference_callback, this, std::placeholders::_1));
    subscription_ControlMode_ = this->create_subscription<std_msgs::msg::Float32>(
        "ControlMode", 10,
        std::bind(&ControlNode::subscription_ControlMode_callback, this, std::placeholders::_1));
    service_setConstraints_ = this->create_service<set_constraints_srv::srv::SetConstraintsSrv>(
        "set_constraints",
        std::bind(&ControlNode::handle_service_setConstraints, this, std::placeholders::_1, std::placeholders::_2));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ControlNode::timer_callback, this));
}

void ControlNode::subscription_PositionReference_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    float pos = position_ + pos_0_;

    if ((pos - msg->data) < 0.01)
    {
        pos = pos + 0.01;
    }
    else if ((pos - msg->data) > 0.01)
    {
        pos = pos - 0.01;
    }
    else
    {
        pos = msg->data;
    }

    position_ = msg->data;
    is_receiving_ = true;
}

void ControlNode::subscription_EffortReference_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    effort_ = msg->data;
    is_receiving_ = true;
    // RCLCPP_INFO(this->get_logger(), "Position: %f", position_);
}

void ControlNode::subscription_ControlMode_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    mode_ = msg->data;
}

void ControlNode::handle_service_setConstraints(
    const std::shared_ptr<set_constraints_srv::srv::SetConstraintsSrv::Request> request,
    std::shared_ptr<set_constraints_srv::srv::SetConstraintsSrv::Response> response)
{
    auto new_max = request->new_maxy;
    auto new_min = request->new_miny;

    GroupCommand group_command(group_->size());

    // Update command and send to the actuator
    for (int i = 0; i < group_->size(); ++i)
    {
        group_command[i].settings().actuator().positionLimitMin().set(new_min);
        group_command[i].settings().actuator().positionLimitMax().set(new_max);
    }
    if (!group_->sendCommandWithAcknowledgement(group_command))
    {
        std::cout << "Did not get acknowledgement from module when sending limits; check connection." << std::endl;
    }
    std::cout << "Set position limit to: " << new_max << std::endl;

    response->success = true;
}

void ControlNode::timer_callback()
{
    time_ += 0.01;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();

    std::thread spin_thread([node]()
                            { rclcpp::spin(node); });

    // Create a Lookup object
    Lookup lookup;

    // Wait 2 seconds for the module list to populate, and then print out its contents
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << std::endl;
    auto entry_list = lookup.getEntryList();
    for (auto entry : *entry_list)
        std::cout
            << "Name: " << entry.name_ << std::endl
            << "Family: " << entry.family_ << std::endl
            << std::endl;

    std::cout << std::endl
              << " NOTE: " << std::endl
              << "  The listing above should show the information for all the modules" << std::endl
              << "  on the local network.  If this is empty make sure that the modules" << std::endl
              << "  are connected, powered on, and that the status LEDs are displaying" << std::endl
              << "  a green soft-fade." << std::endl;

    // Get group
    node->group_ = lookup.getGroupFromNames({"Neurobot"}, {"Codo"});

    GroupFeedback fbk(node->group_->size());
    if (node->group_->getNextFeedback(fbk))
    {
        node->pos_0_ = fbk.getPosition()[0];
    }

    node->group_->setFeedbackFrequencyHz(100);

    if (!node->group_)
    {
        std::cout
            << "Group not found! Check that the family and name of a module on the network" << std::endl
            << "matches what is given in the source file." << std::endl;
        return -1;
    }

    //// Open-loop controller (position)

    // The command struct has fields for various commands and settings; for the
    // actuator, we will primarily use position, velocity, and effort.
    //
    // Fields that are not filled in will be ignored when sending.
    GroupCommand group_command(node->group_->size());
    // GroupCommand uses Eigen types for data interchange
    Eigen::VectorXd positions(1);
    Eigen::VectorXd efforts(1);
    // Allocate feedback
    GroupFeedback group_feedback(node->group_->size());

    double duration = 10; // [sec]
    auto start = std::chrono::system_clock::now();

    std::chrono::duration<double> t(std::chrono::system_clock::now() - start);
    while (rclcpp::ok())
    {
        // Even though we don't use the feedback, getting feedback conveniently
        // limits the loop rate to the feedback frequency
        node->group_->getNextFeedback(group_feedback);

        // Update position set point
        if (node->is_receiving_)
        {
            t = std::chrono::system_clock::now() - start;
            if (node->mode_ == 1)
            {
                positions[0] = node->position_;
                std::cout << positions << std::endl;
                group_command.setPosition(positions);
                node->group_->sendCommand(group_command);
            }
            else
            {
                efforts[0] = node->effort_;
                std::cout << efforts << std::endl;
                group_command.setEffort(efforts);
                node->group_->sendCommand(group_command);
            }
        }
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}