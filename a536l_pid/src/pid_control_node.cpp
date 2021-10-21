#include "BotControl.hpp"
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace botcontrol;

int main(int argc, char** argv){

	ros::init(argc, argv, "pid_control_node_cpp");
	ros::NodeHandle NodeHandle("~");

	BotControl BC(NodeHandle);

	BC.spin();
	
}