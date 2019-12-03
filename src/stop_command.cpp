#include <ros/ros.h>
#include <cstdlib>
#include <std_srvs/Empty.h>
int main(int argc, char **argv){

	ros::init(argc, argv, "stop_reconstructing");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("stop_reconstructing");
	std_srvs::Empty empty;

	if (client.call(empty))
	{
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
	return 0;
}