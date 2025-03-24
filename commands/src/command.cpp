#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "commandNode/command_node.h"


using namespace std;


std::string keyboardInput() {
    std::string line;
    std::getline(std::cin,line);
    return line;
}
void command(std::string line);
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorNodes>();

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
   

    cout<<"over"<<endl;

    rclcpp::shutdown();
    return 0;
}
