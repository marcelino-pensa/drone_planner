

#include "drone_planner/trapezoidal.h"

namespace trapezoidal {

void time_from_displacements(
		const std::vector<double> &cumulative_displacement,
		const double &max_vel, const double &max_acc,
		std::vector<double> *times) {
	// Time to get to max_vel from zero velocity
	const double t_ud = max_vel/max_acc;

	const uint n_points = cumulative_displacement.size();
	const double p0 = cumulative_displacement[0];
	const double pf = cumulative_displacement[n_points-1];
	
	// Find the final time based on displacement, max vel, max acc
	// Also, set some important constants
	double tf, A1, A2, A3;
	double v_top = max_vel;
	bool saturates;
	if (pf > t_ud*max_vel) {  // In case the velocity saturates at max_vel
		tf = t_ud + pf/max_vel;
		A1 = t_ud*max_vel/2.0;
		A2 = (tf-2.0*t_ud)*max_vel;
		A3 = A1;
		saturates = true;
	} else { // If velocity does not saturate at max_vel
		tf = 2.0*sqrt(pf/max_acc);
		v_top = sqrt(pf*max_acc);
		A1 = v_top*tf/4.0;
		A2 = A1;
		saturates = false;
	}

	// For each position in the displacement, calculate time to reach them
	std::vector<double> t;
	for (uint i = 0; i < n_points; i++) {
		const double p = cumulative_displacement[i];

		if (saturates) {
			if (p <= 0) {
				t.push_back(0.0);
			} else if (p < A1) {
				t.push_back(sqrt(2.0*p/max_acc));
			} else if (p < (A1 + A2)) {
				const double Pnew = p - A1;
				const double t2 = Pnew/max_vel;
				t.push_back(t_ud + t2);
			} else if (p < (A1 + A2 + A3)) {
				const double Pnew = p - A1 - A2;
				const double vf = sqrt(max_vel*max_vel - 2.0*max_acc*Pnew);
				const double t3 = 2.0*Pnew/(max_vel + vf);
				t.push_back(tf - t_ud + t3);
			} else {
				t.push_back(tf);
			}
		}

		if (!saturates) {
			if (p <= 0) {
				t.push_back(0.0);
			} else if (p < A1) {
				t.push_back(sqrt(2*p/max_acc));
			} else if (p < (A1 + A2)) {
				const double Pnew = p - A1;
        		const double vf = sqrt(v_top*v_top - 2.0*max_acc*Pnew);
        		const double t2 = 2.0*Pnew/(v_top + vf);
        		t.push_back(0.5*tf + t2);
			} else {
				t.push_back(tf);
			}
		}
	}

	*times = t;
}

void time_from_waypoints(
		const std::vector<geometry_msgs::Point> &waypoints,
		const double &max_vel, const double &max_acc,
		std::vector<double> *times) {
	// Get number of points
	const uint n_points = waypoints.size();

	// Compute cumulative distance along waypoints
	std::vector<double> cumulative_displacement;
	cumulative_displacement.resize(n_points);
	cumulative_displacement[0] = 0.0;
	for (uint i = 1; i < n_points; i++) {
		const Eigen::Vector3d p1(waypoints[i-1].x, waypoints[i-1].y, waypoints[i-1].z);
		const Eigen::Vector3d p2(waypoints[i].x, waypoints[i].y, waypoints[i].z);
		const double dist = (p2 - p1).norm();
		cumulative_displacement[i] = cumulative_displacement[i-1] + dist;
	}

	time_from_displacements(cumulative_displacement, max_vel, max_acc, times);
}

planner_1d::planner_1d (const double init_pos, const double final_pos,
 		                const double &max_vel, const double &max_acc) {
	init_pos_ = init_pos;
	final_pos_ = final_pos;
	final_displacement_ = std::fabs(final_pos_ - init_pos);
	if ((final_pos - init_pos) >= 0) {
		direction_ = 1;
	} else {
		direction_ = -1;
	}
	if (max_acc <= 0.0) {
		std::cout << "Maximum acceleration less than zero! Setting it to 0.1 m/s^2" << std::endl;
		max_acc_ = 0.1;
	} else {
		max_acc_ = max_acc;
	}
	if (max_vel <= 0.0) {
		std::cout << "Maximum velocity less than zero! Setting it to 0.1 m/s^2" << std::endl;
		max_vel_ = 0.1;
	} else {
		max_vel_ = max_vel;
	}
	t_up_down_ = max_vel_/max_acc_;

	// Final time is calculated based on whether the velocity saturates or not
	if (final_displacement_ > t_up_down_*max_vel_) {
		final_time_ = t_up_down_ + final_displacement_/max_vel_;
	} else {
		final_time_ = 2*sqrt(final_displacement_/max_acc_);
	}
}

void planner_1d::get_pva_at_time (const double &time, double *pos,
	                              double *vel, double *acc) {
	if (time <= 0) {
		*pos = init_pos_;
		*vel = 0.0;
		*acc = 0.0;
	} else if (time >= final_time_) {
		*pos = final_pos_;
		*vel = 0.0;
		*acc = 0.0;
	} else if (t_up_down_ < final_time_/2.0) {  // If velocity gets to saturate at maximum value
		if (time <= t_up_down_) {  // Velocity profile still going up
			*acc = max_acc_ * direction_;
			*vel = max_acc_ * time * direction_;
			*pos = init_pos_ + 0.5*max_acc_*time*time * direction_;
		} else if (time <= final_time_ - t_up_down_) {  // Velocity saturated at maximum
			*acc = 0.0;
			*vel = max_vel_ * direction_;
			const double dt = time - t_up_down_;
			const double displacement = max_vel_*dt + 0.5*max_acc_*t_up_down_*t_up_down_;
			*pos = init_pos_ + displacement * direction_;
		} else {  // Velocity profile slowing down
			*acc = - max_acc_ * direction_;
			*vel = max_acc_ * (final_time_ - time) * direction_;
			const double dt = time - (final_time_ - t_up_down_);
			const double displacement = 0.5*max_acc_*t_up_down_*t_up_down_ + 
										max_vel_*(final_time_ - 2.0*t_up_down_) +
										max_vel_*dt - 0.5*max_acc_*dt*dt;
			*pos = init_pos_ + displacement * direction_;
		}
	} else {  // If velocity does not get to saturate at maximum value
		if (time <= final_time_/2.0) {
			*acc = max_acc_ * direction_;
			*vel = max_acc_ * time * direction_;
			*pos = init_pos_ + (0.5*max_acc_*time*time) * direction_;
		} else {
			*acc = - max_acc_ * direction_;
			*vel = (final_time_ - time)*max_acc_ * direction_;
			const double half_time = final_time_/2.0;
			const double dt = time - half_time;
			const double top_velocity = max_acc_*half_time;
			const double displacement = 0.5*max_acc_*half_time*half_time +
										top_velocity*dt - 
										0.5*max_acc_*dt*dt;
			*pos = init_pos_ + displacement * direction_;
		}
	}
}

void planner_1d::sample_trajectory (const double &sample_freq,
									const double &final_time,
				                    std::vector<double> *time_hist,
									std::vector<double> *pos_hist,
									std::vector<double> *vel_hist,
									std::vector<double> *acc_hist) {
	double time = 0.0;
	const double dt = 1.0/sample_freq;
	double pos, vel, acc;
	while (time < final_time) {
		this->get_pva_at_time(time, &pos, &vel, &acc);
		time_hist->push_back(time);
		pos_hist->push_back(pos);
		vel_hist->push_back(vel);
		acc_hist->push_back(acc);
		time = time + dt;
	}
	time = final_time;
	this->get_pva_at_time(time, &pos, &vel, &acc);
	time_hist->push_back(time);
	pos_hist->push_back(pos);
	vel_hist->push_back(vel);
	acc_hist->push_back(acc);
}

void planner_1d::sample_trajectory (const std::vector<double> &time_hist,
		 							std::vector<double> *pos_hist,
		 							std::vector<double> *vel_hist,
		 							std::vector<double> *acc_hist) {
	double pos, vel, acc;
	for (uint i = 0; i < time_hist.size(); i++) {
		this->get_pva_at_time(time_hist[i], &pos, &vel, &acc);		
		pos_hist->push_back(pos);
		vel_hist->push_back(vel);
		acc_hist->push_back(acc);
	}
}

// void planner_1d::plot_traj (const std::vector<double> &time_hist,
// 				            const std::vector<double> &pos_hist,
// 							const std::vector<double> &vel_hist,
// 							const std::vector<double> &acc_hist) {
// 	// Use gnuplot for plotting
//     Gnuplot gp;
//     gp << "plot '-' using 1:2 with lines title 'Position'";
//     gp << ", '-' using 1:2 with lines title 'Velocity'";
//     gp << ", '-' using 1:2 with lines title 'Acceleration'";
//     gp << std::endl;
//     gp.send1d(boost::make_tuple(time_hist, pos_hist));
//     gp.send1d(boost::make_tuple(time_hist, vel_hist));
//     gp.send1d(boost::make_tuple(time_hist, acc_hist));
//     gp << "set grid" << std::endl;
//     gp << "replot" << std::endl;
// }

planner_3d::planner_3d (const Eigen::Vector3d &init_pos,
 		                const Eigen::Vector3d &final_pos,
 		                const double &max_vel, const double &max_acc) {
	init_pos_ = init_pos;
	final_pos_ = final_pos;
	const double final_displacement_ = (final_pos - init_pos).norm();
	direction_ = (final_pos - init_pos).normalized();

	linear_planner = planner_1d(0.0, final_displacement_, max_vel, max_acc);
}

void planner_3d::get_pva_at_time (const double &time,
 		Eigen::Vector3d *pos, Eigen::Vector3d *vel, Eigen::Vector3d *acc) {
	double pos_linear, vel_linear, acc_linear;
	linear_planner.get_pva_at_time(time, &pos_linear, &vel_linear, &acc_linear);
	*pos = init_pos_ + pos_linear * direction_;
	*vel = vel_linear * direction_;
	*acc = acc_linear * direction_;
}

void planner_3d::sample_trajectory (const double &sample_freq,
									const double &final_time,
 		                            std::vector<double> *time_hist,
 					      		    std::vector<Eigen::Vector3d> *pos_hist,
 							        std::vector<Eigen::Vector3d> *vel_hist,
 							        std::vector<Eigen::Vector3d> *acc_hist) {
	double time = 0.0;
	const double dt = 1.0/sample_freq;
	// const double final_time = this->get_final_time();
	Eigen::Vector3d pos, vel, acc;
	// Eigen::Vector3d direction = (final_pos - init_pos).normalized();
	std::vector<double> t_hist, pos_, vel_, acc_;
	while (time < final_time) {
		this->get_pva_at_time(time, &pos, &vel, &acc);
		time_hist->push_back(time);
		pos_hist->push_back(pos);
		vel_hist->push_back(vel);
		acc_hist->push_back(acc);
		time = time + dt;
	}
	time = final_time;
	this->get_pva_at_time(time, &pos, &vel, &acc);
	time_hist->push_back(time);
	pos_hist->push_back(pos);
	vel_hist->push_back(vel);
	acc_hist->push_back(acc);
}

void planner_3d::sample_trajectory (const std::vector<double> &time_hist,
		 							std::vector<Eigen::Vector3d> *pos_hist,
		 							std::vector<Eigen::Vector3d> *vel_hist,
		 							std::vector<Eigen::Vector3d> *acc_hist) {
	Eigen::Vector3d pos, vel, acc;
	for (uint i = 0; i < time_hist.size(); i++) {
		this->get_pva_at_time(time_hist[i], &pos, &vel, &acc);		
		pos_hist->push_back(pos);
		vel_hist->push_back(vel);
		acc_hist->push_back(acc);
	}
}

// void planner_3d::plot_traj_1D (const std::vector<double> &time_hist,
// 				         	   const std::vector<Eigen::Vector3d> &pos_hist,
// 						       const std::vector<Eigen::Vector3d> &vel_hist,
// 						       const std::vector<Eigen::Vector3d> &acc_hist) {
// 	std::vector<double> p_hist, v_hist, a_hist;
// 	for (uint i = 0; i < time_hist.size(); i++) {
// 		p_hist.push_back((pos_hist[i]-init_pos_).dot(direction_));
// 		v_hist.push_back(vel_hist[i].dot(direction_));
// 		a_hist.push_back(acc_hist[i].dot(direction_));
// 	}

// 	// Use gnuplot for plotting
//     Gnuplot gp;
//     gp << "plot '-' using 1:2 with lines title 'Position'";
//     gp << ", '-' using 1:2 with lines title 'Velocity'";
//     gp << ", '-' using 1:2 with lines title 'Acceleration'";
//     gp << std::endl;
//     gp.send1d(boost::make_tuple(time_hist, p_hist));
//     gp.send1d(boost::make_tuple(time_hist, v_hist));
//     gp.send1d(boost::make_tuple(time_hist, a_hist));
//     gp << "set grid" << std::endl;
//     gp << "replot" << std::endl;
// }



}  // namespace trapezoidal