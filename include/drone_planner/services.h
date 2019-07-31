#ifndef DRONE_PLANNER_H_
#define DRONE_PLANNER_H_

#include "ros/ros.h"

// Library that solves trapezoidal trajectories
#include "drone_planner/trapezoidal.h"

// Message/service types
#include "drone_planner/trapezoidal_p2p.h"
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

 private:
 	ros::NodeHandle *nh_;
	ros::ServiceServer trapezoidal_srv;
	visualization::TrajPublisher traj_pub_obj;

	drone_planner::PVA_4d constuct_PVA (const Eigen::Vector3d &pos,
				                        const Eigen::Vector3d &vel,
				                        const Eigen::Vector3d &acc,
				                        const double &yaw, const double &yaw_vel,
				                        const double &yaw_acc, const double &time);
};

}  // namespace planner

#endif  // DRONE_PLANNER_H_