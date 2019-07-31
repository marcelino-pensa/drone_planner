#include "ros/ros.h"
#include <drone_planner/services.h>
#include <drone_planner/trapezoidal_p2p_action.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "drone_planner");
	ros::NodeHandle node("~");

	// Start services
	planner::ServicesClass services(&node);

	// Start action server
  	trapezoidal_p2pAction trapezoidal_p2pAction("trapezoidal_p2p_action");

	ros::spin();

	return 0;
}