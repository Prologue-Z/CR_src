#include <ros/ros.h>
#include <CANTest/CANMotor.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CANMotor_Node");	//初始化ROS，节点名命名为node_a，节点名必须保持唯一
    ros::NodeHandle nh("~");//实例化节点, 节点进程句柄
    CANMotor_NS::CANMotor MotorTest(nh);

    signed short velocity[3] = {1000,1000,1000};
    
    MotorTest.SetModeID(2);
    MotorTest.SetVelocity(velocity);

    
    

}