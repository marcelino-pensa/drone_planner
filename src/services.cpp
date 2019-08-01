
#include "drone_planner/services.h"

namespace planner {

ServicesClass::ServicesClass(ros::NodeHandle *nh) {
	nh_ = nh;
	trapezoidal_srv = nh->advertiseService("trapezoidal_solver", &ServicesClass::trapezoidal_service, this);
	min_time_srv = nh_->advertiseService("min_time_solver", &ServicesClass::min_time_service, this);
	ROS_INFO("[drone_planner] Created service %s", trapezoidal_srv.getService().c_str());
	ROS_INFO("[p4_services] Created service %s", min_time_srv.getService().c_str());
	traj_pub_obj = visualization::TrajPublisher(nh_);
}

bool ServicesClass::trapezoidal_service(drone_planner::trapezoidal_p2p::Request  &req,
	                                    drone_planner::trapezoidal_p2p::Response &res) {
	const Eigen::Vector3d init_pos(req.init_pos.x, req.init_pos.y, req.init_pos.z);
	const Eigen::Vector3d final_pos(req.final_pos.x, req.final_pos.y, req.final_pos.z);
	const double max_vel = req.max_vel;
	const double max_acc = req.max_acc;
	trapezoidal::planner_3d tp_xyz(init_pos, final_pos, max_vel, max_acc);

	const double init_yaw = req.init_yaw;
	const double final_yaw = req.final_yaw;
	const double max_yaw_vel = req.max_yaw_vel;
	const double max_yaw_acc = req.max_yaw_acc;
	trapezoidal::planner_1d tp_yaw(init_yaw, final_yaw, max_yaw_vel, max_yaw_acc);

	res.final_time = std::max(tp_xyz.get_final_time(), tp_yaw.get_final_time());

	const double sample_freq = req.sampling_freq;
	std::vector<double> time_hist;
	std::vector<Eigen::Vector3d> pos_hist, vel_hist, acc_hist;
	std::vector<double> yaw_hist, yaw_vel_hist, yaw_acc_hist;
	tp_xyz.sample_trajectory(sample_freq, res.final_time, &time_hist,
		                     &pos_hist, &vel_hist, &acc_hist);
	tp_yaw.sample_trajectory(time_hist, &yaw_hist, &yaw_vel_hist, &yaw_acc_hist);

	// Populate output pva structure
	for (uint i = 0; i < time_hist.size(); i++) {
		res.pva_vec.push_back(helper::constuct_PVA(
			pos_hist[i], vel_hist[i], acc_hist[i], yaw_hist[i],
			yaw_vel_hist[i], yaw_acc_hist[i], time_hist[i]));
	}

	// Publish trajectory to be seen in Rviz
    traj_pub_obj.VisualizePath(pos_hist);

	if (req.visualize_output) {
		// tp_xyz.plot_traj_1D(time_hist, pos_hist, vel_hist, acc_hist);
		// tp_yaw.plot_traj(time_hist, yaw_hist, yaw_vel_hist, yaw_acc_hist);
		traj_pub_obj.PubRealTimeTraj(res.pva_vec, sample_freq, res.final_time);
	}
}

bool ServicesClass::min_time_service(drone_planner::min_time::Request  &req,
                                     drone_planner::min_time::Response &res) {
	ROS_WARN("[drone_planner] Service not compiled! \"Set COMPILE_MIN_TIME_SERVICE = true\" in CMakeLists.txt!");
	res.final_time = -1.0;
	return false;
}

}  // namespace planner