#include "main/main_node.hpp"

MainNode::MainNode()
    : Node("main_node"), frequency_(0.0), amplitude_(0.0), is_pulse_(false), time_(0.0), mode_(1),
      disturb_ampl_(0.0), disturb_duration_(0.0), disturb_interval_(0.0), last_disturb_time_(0.0)
{
    publisher_PositionReference = this->create_publisher<std_msgs::msg::Float32>("PositionReference", 10);
    publisher_CleanSignal = this->create_publisher<std_msgs::msg::Float32>("CleanSignal", 10);
    publisher_EffortReference = this->create_publisher<std_msgs::msg::Float32>("EffortReference", 10);
    publisher_ControlMode = this->create_publisher<std_msgs::msg::Float32>("ControlMode", 10);
    subscription_SliderParameters = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "SliderParameters", 10,
        std::bind(&MainNode::subscription_SliderParameters_callback, this, std::placeholders::_1));
    std_msgs::msg::Float32 message;
    message.data = 1;
    publisher_ControlMode->publish(message);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MainNode::timer_callback, this));

    client_ = this->create_client<set_constraints_srv::srv::SetConstraintsSrv>("set_constraints");
}

void MainNode::subscription_SliderParameters_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    frequency_ = msg->data[0];
    amplitude_ = msg->data[1];
    disturb_ampl_ = msg->data[2];
    disturb_duration_ = msg->data[3];
    disturb_interval_ = msg->data[4];
}

void MainNode::timer_callback()
{
    static LowPassFilter outputFilter(0.2);

    auto clean_message = std_msgs::msg::Float32();
    auto combined_message = std_msgs::msg::Float32();
    float rawValue;
    float disturb;
    double time_since_last_disturb = time_ - last_disturb_time_;

    if (time_since_last_disturb >= disturb_interval_)
    {
        if (time_since_last_disturb < (disturb_duration_ + disturb_interval_))
        {
            disturb = disturb_ampl_;
        }
        else
        {
            disturb = 0;

            last_disturb_time_ = time_;
        }
    }
    else
    {
        disturb = 0.0;
    }

    if (mode_ == 1)
    {
        if (is_pulse_)
        {
            rawValue = (std::fmod(2 * M_PI * frequency_ * time_, 2 * M_PI) < M_PI) ? amplitude_ : -amplitude_;
        }
        else
        {
            rawValue = amplitude_ * std::sin(2 * M_PI * frequency_ * time_);
        }

        clean_message.data = outputFilter.process(rawValue);
        combined_message.data = rawValue + disturb + pos_offset_;
        publisher_CleanSignal->publish(clean_message);
        publisher_PositionReference->publish(combined_message);
    }
    else if (mode_ == 2)
    {
        double freq = frequency_ * 2.0 * M_PI;
        clean_message.data = amplitude_ * std::sin(freq * time_) + disturb;
        publisher_EffortReference->publish(clean_message);
    }
    time_ += 0.01;
}

void MainNode::call_service()
{
    auto request = std::make_shared<set_constraints_srv::srv::SetConstraintsSrv::Request>();
    request->new_maxy = new_maxy_;
    request->new_miny = new_miny_;
    request->new_delta_y = new_delta_y_;
    request->new_minu = new_minu_;
    request->new_maxu = new_maxu_;
    request->new_delta_u = new_delta_u_;
    request->new_alpha = new_alpha_;
    request->new_constraint = new_constraint_;

    RCLCPP_INFO(this->get_logger(), "Calling service with values:");
    RCLCPP_INFO(this->get_logger(), "new_maxy: %f", request->new_maxy);
    RCLCPP_INFO(this->get_logger(), "new_miny: %f", request->new_miny);

    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, std::chrono::seconds(10)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Service call succeeded");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
}

std::string keyboardInput()
{
    std::string line;
    std::getline(std::cin, line);
    return line;
}

void MainNode::command(std::string line)
{
    size_t pos;
    float numericValue;
    char aux = line[0];
    if (line == "SINWAVE" || line == "PULSE" || line == "POSITION" || line == "EFFORT" || line == "HELP")
    {
        aux = 'a';
    }

    switch (aux)
    {
    case 'F':
        numericValue = std::stof(line.substr(1), &pos);
        frequency_ = numericValue;
        break;
    case 'A':
        numericValue = std::stof(line.substr(1), &pos);
        amplitude_ = numericValue;
        break;
    case 'M':
        numericValue = std::stof(line.substr(1), &pos);
        new_maxy_ = numericValue;
        call_service();
        break;
    case 'm':
        numericValue = std::stof(line.substr(1), &pos);
        new_miny_ = numericValue;
        call_service();
        break;
    case 'D':
        numericValue = std::stof(line.substr(1), &pos);
        new_delta_y_ = numericValue;
        call_service();
        break;
    default:
        if (line == "SINWAVE")
        {
            is_pulse_ = false;
        }
        else if (line == "PULSE")
        {
            is_pulse_ = true;
        }
        else if (line == "POSITION")
        {
            std_msgs::msg::Float32 message;
            mode_ = 1;
            message.data = 1;
            publisher_ControlMode->publish(message);
        }
        else if (line == "EFFORT")
        {
            std_msgs::msg::Float32 message;
            mode_ = 2;
            message.data = 2;
            publisher_ControlMode->publish(message);
        }
        else if (line == "HELP")
        {
            std::cout << "Modes: SINWAVE," << std::endl;
        }
        else
        {
            std::cout << "Not a valid Command" << std::endl;
        }
        break;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();
    std::string line2;
    auto future = std::async(std::launch::async, keyboardInput);
    while (true)
    {
        rclcpp::Rate loopRate(100);

        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            auto line = future.get();
            future = std::async(std::launch::async, keyboardInput);
            node->command(line);
        }
        rclcpp::spin_some(node);
        loopRate.sleep();
    }

    std::cout << "over" << std::endl;

    rclcpp::shutdown();
    return 0;
}