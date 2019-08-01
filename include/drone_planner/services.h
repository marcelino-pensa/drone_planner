#ifndef DRONE_PLANNER_SERVICES_H_
#define DRONE_PLANNER_SERVICES_H_

#include "ros/ros.h"

// Local libraries
#include "drone_planner/helper.h"

// Library that solves trapezoidal trajectories
#include "drone_planner/trapezoidal.h"

// Message/service types
#include "drone_planner/trapezoidal_p2p.h"
#include "drone_planner/min_time.h"
#include "drone_planner/PVA_4d.h"

// Ros message types
#include "drone_planner/visualization_functions.h"

namespace planner {

class ServicesClass {
 public:
	ServicesClass(ros::NodeHandle *nh);
	~ServicesClass() { };

	bool trapezoidal_service(drone_planner::trapezoidal_p2p::Request  &req,
	                         drone_planner::trapezoidal_p2p::Response &res);
	bool min_time_service(drone_planner::min_time::Request  &req,
	                      drone_planner::min_time::Response &res);

 private:
 	ros::NodeHandle *nh_;
	ros::ServiceServer min_time_srv, trapezoidal_srv;
	visualization::TrajPublisher traj_pub_obj;
};

}  // namespace planner

#endif  // DRONE_PLANNER_SERVICES_H_