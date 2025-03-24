#include "command_node.h"


//Funcion para procesamiento y publicacion

// Callback ejecutado cada cierto tiempo (eliminar si no se usa)
void MotorNodes::timer_callback()
{


}

void MotorNodes::PublishPosReference(double reference){
    std_msgs::msg::Float32 data1;
    data1.data=reference;
    publisher_posReference->publish(data1);
}
void MotorNodes::PublishVelReference(double reference){
    std_msgs::msg::Float32 data1;
    data1.data=reference;
    publisher_velReference->publish(data1);
}
void MotorNodes::PublishCommand(double command){
    std_msgs::msg::Float32 data1;
    data1.data=command;
    publisher_ControlMode->publish(data1);
}
void MotorNodes::command(std::string line){
    size_t pos;
    float numericValue;
    char aux=line[0];
    if (line=="s" || line=="VELOCITY" || line =="POSITION" || line =="MPC" || line =="VERBOSE" ){
        aux='a';
    }

    std::cout<<"primercarater es: "<<aux<<std::endl;
    switch (aux) {
    case 'P':

        numericValue = std::stof(line.substr(1), &pos);
        this->PublishPosReference(numericValue);

        break;

    case 'V':
        numericValue = std::stof(line.substr(1), &pos);
        this->PublishVelReference(numericValue);
        break;
    default:
        if (line=="POSITION")
        {this->PublishCommand(1);}
        else if (line=="VELOCITY")
        {this->PublishCommand(2);}
         else if (line=="MPC")
        {this->PublishCommand(5);}
        else if (line=="s")
        {this->PublishCommand(6);}
        else if (line=="VERBOSE")
        {this->PublishCommand(7);}
        else{
            std::cout<<"Not a valid Command"<<std::endl;
        }
        break;
    }

}



