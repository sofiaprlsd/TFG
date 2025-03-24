#ifndef ZMPNODE_H
#define ZMPNODE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <iostream>


using namespace std::chrono_literals;

//Estructura compartida para almacenar datos al recibir cada topic
struct SharedData {

    std_msgs::msg::Float32 COMx;
    std_msgs::msg::Float32 COMy;

};
class MotorNodes : public rclcpp::Node
{
public:
    MotorNodes() : Node("control_ZMP"){
        //Possible internal parameters
        this->declare_parameter("param", 1);
        this->declare_parameter("param2", 2);
        // Create a subscribers
        //This subscriber can e used to read the pelvis/COM from dynamic modeling
        
        //This subscriber read sensor state from the ST, This probably changes to used simple messages
        /*subscriber_framestate = create_subscription<nimble_interfaces::msg::FrameState>(
                    "frame_state", 10,
                    [this](const nimble_interfaces::msg::FrameState msg) {
            // Callback function
            call_back_framestate(msg);
        });
*/
        // Create a publishers

        //Control Publishers
        publisher_posReference = create_publisher<std_msgs::msg::Float32>("PositionReference", 10);
        publisher_velReference = create_publisher<std_msgs::msg::Float32>("SpeedReference", 10);
        publisher_ControlMode  = create_publisher<std_msgs::msg::Float32>("ControlMode", 10);

        //Create wall timer to publish periodically (eliminar si no se usa)
  //      timer_ = this->create_wall_timer(1000ms, std::bind(&ZMPNode::timer_callback, this));

    };
    SharedData	shared_data_;    //estructura de datos
    //********************************************//
    //**************SUBSCRIBERS *****************/
    //********************************************//


   // rclcpp::Subscription<nimble_interfaces::msg::FrameState>::SharedPtr subscriber_framestate;

    //********************************************//
    //*************PUBLISHERS*******************************//
    //********************************************//
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_posReference;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_velReference;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_ControlMode;
    rclcpp::TimerBase::SharedPtr timer_; //timer (eliminar si no se usa)


    //Callbacks, funciones asociadas a la recepcion de cada topic
    //void call_back_framestate(const nimble_interfaces::msg::FrameState & frame_state_msg);

    //Funcion para procesamiento y publicacion
    void timer_callback();
public:
    void PublishPosReference(double reference);
    void PublishVelReference(double reference);
    void PublishCommand(double command);
    void command(std::string line);
};

#endif // ZMPNODE_H
