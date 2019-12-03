#include <ros/ros.h>
#include <cstdlib>
#include <std_srvs/Empty.h>
#include "colored_mesh_msgs/RetreiveReconstruction.h"
int main(int argc, char **argv){

	ros::init(argc, argv, "retreive_reconstruction");
	ros::NodeHandle n;
	ros::ServiceClient client =
			n.serviceClient<colored_mesh_msgs::RetreiveReconstruction>("retreive_reconstruction");
	colored_mesh_msgs::RetreiveReconstruction reconstruction;

	if (client.call(reconstruction))
	{
		std::cout << "vertex count " << reconstruction.response.mesh.vertices.size() << std::endl;
	}
	else
	{
		ROS_ERROR("Failed to call service retreive_reconstructing");
		return 1;
	}
	return 0;
}