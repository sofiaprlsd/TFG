#include "../include/neurobot_graphics/main_node.hpp"

MainNode::MainNode()
    : Node("main_node"), frequency_(1.0), amplitude_(1.0), is_pulse_(false), time_(0.0)
{
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("PositionReference", 10);
    publisher_ControlMode  = this->create_publisher<std_msgs::msg::Float32>("ControlMode", 10);
    std_msgs::msg::Float32 data1;
    data1.data=1;
    publisher_ControlMode->publish(data1);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&MainNode::timer_callback, this));

    client_ = this->create_client<set_constraints_srv::srv::SetConstraintsSrv>("set_constraints_srv");
}

void MainNode::timer_callback()
{
    auto message = std_msgs::msg::Float32();
    if(is_pulse_){
        message.data = (std::fmod(2 * M_PI * frequency_ * time_, 2 * M_PI) < M_PI) ? amplitude_ : -amplitude_;
    } else {
        message.data = amplitude_ * std::sin(2 * M_PI * frequency_ * time_);
    }
    time_ += 0.01;
    publisher_->publish(message);
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

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Service call succeeded");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }
}

std::string keyboardInput() {
    std::string line;
    std::getline(std::cin,line);
    return line;
}

void MainNode::command(std::string line){
    size_t pos;
    float numericValue;
    char aux=line[0];
    if (line=="SINWAVE" || line =="PULSE" || line =="HELP"){
        aux='a';
    }

    switch (aux) {
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
        new_miny_ = numericValue;
        call_service();
        break;
    default:
        if (line=="SINWAVE")
        {is_pulse_ = false;}
        else if (line=="PULSE")
        {is_pulse_ = true;}
        else if (line=="HELP")
        {std::cout<<"Modes: SINWAVE,"<<std::endl;}
        else{
            std::cout<<"Not a valid Command"<<std::endl;
        }
        break;
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();

    double steps=-1;
    std::string line2;
    auto future = std::async(std::launch::async, keyboardInput);
    while (true){
        rclcpp::Rate loopRate(100);

        if(future.wait_for(std::chrono::seconds(0))==std::future_status::ready){
            auto line=future.get();
            future=std::async(std::launch::async,keyboardInput);
            node->command(line);

        }
            rclcpp::spin_some(node);
            loopRate.sleep();

        }
   

    std::cout<<"over"<<std::endl;

    rclcpp::shutdown();
    return 0;
}