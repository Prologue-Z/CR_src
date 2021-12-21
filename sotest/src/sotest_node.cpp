#include <ros/ros.h>
#include "ECanVci.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sotest_node");	//初始化ROS，节点名命名为node_a，节点名必须保持唯一
    ros::NodeHandle nh("~");//实例化节点, 节点进程句柄
    

    int nDeviceType = 3;    /* PCIe-9221 */ 
	int nDeviceInd = 0;     /* 索引号0 */ 
	int nReserved = 0;    
	DWORD dwRel; 
	 
	dwRel = OpenDevice(nDeviceType, nDeviceInd, nReserved); 
	if (dwRel != STATUS_OK) 
	{ 
	    ROS_ERROR("fail open dev"); 
	    return -1; 
	} 
	ROS_INFO("open succ");    
    
	CloseDevice(nDeviceType, nDeviceInd);

}
