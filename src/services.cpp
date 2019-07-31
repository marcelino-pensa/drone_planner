
#include "drone_planner/services.h"

namespace planner {

ServicesClass::ServicesClass(ros::NodeHandle *nh) {
	nh_ = nh;
	trapezoidal_srv = nh->advertiseService("trapezoidal_solver", &ServicesClass::trapezoidal_service, this);
	ROS_INFO("[drone_planner] Created service %s", trapezoidal_srv.getService().c_str());
	traj_pub_obj = visualization::TrajPublisher(nh_);
}

bool  ServicesClass::trapezoidal_service(drone_planner::trapezoidal_p2p::Request  &req,
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
		res.pva_vec.push_back(this->constuct_PVA(
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

drone_planner::PVA_4d ServicesClass::constuct_PVA (const Eigen::Vector3d &pos,
							                       const Eigen::Vector3d &vel,
							                       const Eigen::Vector3d &acc,
							                       const double &yaw, const double &yaw_vel,
							                       const double &yaw_acc, const double &time) {
	drone_planner::PVA_4d pva;
	pva.pos.x = pos[0]; pva.pos.y = pos[1]; pva.pos.z = pos[2];
	pva.vel.x = vel[0]; pva.vel.y = vel[1]; pva.vel.z = vel[2];
	pva.acc.x = acc[0]; pva.acc.y = acc[1]; pva.acc.z = acc[2];
	pva.yaw = yaw;
	pva.yaw_vel = yaw_vel;
	pva.yaw_acc = yaw_acc;
	pva.time = time;
	return pva;
}

}  // namespace planner