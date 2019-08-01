
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
	ROS_INFO("[drone_planner] Running minimum time solver!");

	// Problem variables
	std::vector<double> times;
	std::vector<p4::NodeEqualityBound> node_eq;
	std::vector<p4::SegmentInequalityBound> segment_ineq;
	p4::PolynomialSolver::Options solver_options;
	std::vector<p4::NodeInequalityBound> node_ineq;  // We leave this structure empty in the current service

	// First we check whether the trajectory seems to be a straight line
	// Straight lines are dealt differently than non-straight lines
	bool is_straight = p4_helper::is_trajectory_straight(req.pos_array);

	// Setup optimization problem
	p4_helper::setup_min_time_problem(req, is_straight,
			&times, &node_eq, &segment_ineq, &solver_options);

	// Solve problem
	// ros::Time t0 = ros::Time::now();
	// const p4::PolynomialPath path =
	//     solver.Run(times, node_eq, node_ineq, segment_ineq);

	// Find path through gradient descent optimizing segment times
	ros::Time t1 = ros::Time::now();
	p4::PolynomialPath path_optimized;
	std::vector<double> times_final;
	p4_helper::solve_initial_min_time_trajectory(
		times, node_eq, segment_ineq, solver_options,
		node_ineq, &times_final, &path_optimized);

	ros::Time t2 = ros::Time::now();
	ROS_WARN("[drone_planner] Trajectory generation time: %f", (t2-t1).toSec());

	// Time optimizer fails sometimes, so we set a maximum number of attempts
	// For each attempt, we change the initial polynomial final time that we feed to it
	const uint max_attempts = 3;

	// Time optimizer parameters
	const uint n_coeff = path_optimized.coefficients[0].col(0).size();
	const uint poly_order = n_coeff - 1;
	double d_s = 0.02, rho = 0.0;
	p4::PolynomialSolver solver(solver_options);
	Eigen::VectorXd segment_times;
	Eigen::MatrixXd coeff_matrix;
	uint n_iterations = 0;
	while(1) {
		n_iterations = n_iterations + 1;

		// Convert segments from Tucker-polynomials (used in P4) into the form expected by the minimum time solver
		// Defining dt = t_seg_final - t_seg_initial, Tucker polynomials are written as follows:
		// p(t) = [a0 a1 a2 ... a3]*[1 t/dt t^2/(2! dt^2) t^3/(3! dt^3) ... t^n/(n! dt^n)]
		p4_helper::tucker_polynomials_to_coeff_matrix(
				path_optimized, times_final, &segment_times, &coeff_matrix);

		// Call the time optimizer solver
		time_optimizer::TimeOptimizerClass time_optimizer_obj(req.max_vel, req.max_acc, req.max_jerk,
	        d_s, rho, poly_order, req.sampling_freq, coeff_matrix, segment_times,
	        req.visualize_output, &res.pva_vec, &res.final_time);

		if ((res.final_time > 0.0) || (n_iterations == max_attempts)) {
			break;
		}

		// If the time optimizer fails, we try again with new polynomial changing the segment times
		times_final = helper::scale_std_vector(times_final, 1.5);
		solver.Run(times_final, node_eq, node_ineq, segment_ineq);
		path_optimized = solver.Run(times_final, node_eq, node_ineq, segment_ineq);
	}


	// Test if optimization was successful 
	if (res.final_time <= 0.0) {
		ROS_WARN("Time Optimization was not Successful!");
		for (uint i = 0; i < req.pos_array.size(); i++) {
			std::cout << req.pos_array[i].x << " "
			          << req.pos_array[i].y << " "
			          << req.pos_array[i].z << " "
			          << times_final[i] << std::endl;
		}
		// p4_helper::plot_results(times_final, path_optimized);
		res.final_time = -1.0;
	} else {
		// visualize the spatial fixed trajectory in rviz
    	traj_pub_obj.VisualizePath(coeff_matrix, segment_times);
    	// traj_pub_obj.plot_results_gnuplot(res.pva_vec);

	    // Publish a "real-time" visualization of the trajectory
	    if (req.visualize_output) {
	    	// traj_pub_obj.plot_results_gnuplot(res.pva_vec);
			traj_pub_obj.PubRealTimeTraj(res.pva_vec, req.sampling_freq, res.final_time);
	    }
	}

	return true;
}

}  // namespace planner