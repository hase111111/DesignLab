#include "define.h"

#ifdef DESIGNLAB_USE_ROS

// ROSを使用する場合はここが有効化される

#include <ros/ros.h>

int main(int argc, char** argv)
{
	//ros::init(argc, argv, "designlab");

	//ros::NodeHandle nh;

	//ros::Rate loop_rate(10);

	//while (ros::ok())
	//{
	//	ros::spinOnce();

	//	loop_rate.sleep();
	//}

	return 0;
}

#endif 