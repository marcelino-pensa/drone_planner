
#ifndef TRAPEZOIDAL_HELPER_H_
#define TRAPEZOIDAL_HELPER_H_

#include "geometry_msgs/Point.h"
#include <Eigen/Dense>
// #include "gnuplot-iostream.h"

namespace trapezoidal {

// Velocity profile
//
// v_max|    ___________________
//      |   /'                 '\
//      |  / '                 ' \
//    v0| /  '                 '  \
//      |/   '                 '   \
//       ----'-----------------'---------
//      0   t_up_down              tf

void time_from_displacements(
		const std::vector<double> &cumulative_displacement,
		const double &max_vel, const double &max_acc,
		std::vector<double> *times);

void time_from_waypoints(
		const std::vector<geometry_msgs::Point> &waypoints,
		const double &max_vel, const double &max_acc,
		std::vector<double> *times);

class planner_1d {
	double final_displacement_, max_vel_, max_acc_, final_time_;
	double init_pos_, final_pos_;
	int direction_;  // It can be positive or negative
 	double t_up_down_;  // time for velocity to go from zero to maximum

 public:
 	planner_1d() { }
 	planner_1d (const double init_pos, const double final_pos,
 		        const double &max_vel, const double &max_acc);

 	double get_final_time () { return final_time_; }

 	void get_pva_at_time (const double &time, double *pos,
 		                  double *vel, double *acc);

 	void sample_trajectory (const double &sample_freq,
 							const double &final_time,
 		                    std::vector<double> *time_hist,
 							std::vector<double> *pos_hist,
 							std::vector<double> *vel_hist,
 							std::vector<double> *acc_hist);
 	void sample_trajectory (const std::vector<double> &time_hist,
 							std::vector<double> *pos_hist,
 							std::vector<double> *vel_hist,
 							std::vector<double> *acc_hist);

 	// void plot_traj (const std::vector<double> &time_hist,
		//             const std::vector<double> &pos_hist,
		// 			const std::vector<double> &vel_hist,
		// 			const std::vector<double> &acc_hist);
};

class planner_3d {
 private:
 	planner_1d linear_planner;
 	Eigen::Vector3d init_pos_, final_pos_, direction_;

 public:
 	planner_3d() { }
 	planner_3d (const Eigen::Vector3d &init_pos,
 		     const Eigen::Vector3d &final_pos,
 		     const double &max_vel, const double &max_acc);

 	double get_final_time () { return linear_planner.get_final_time(); }

 	void get_pva_at_time (const double &time,
 		Eigen::Vector3d *pos, Eigen::Vector3d *vel, Eigen::Vector3d *acc);

 	void sample_trajectory (const double &sample_freq, 
 							const double &final_time,
 		                    std::vector<double> *time_hist,
 							std::vector<Eigen::Vector3d> *pos_hist,
 							std::vector<Eigen::Vector3d> *vel_hist,
 							std::vector<Eigen::Vector3d> *acc_hist);
 	void sample_trajectory (const std::vector<double> &time_hist,
 							std::vector<Eigen::Vector3d> *pos_hist,
 							std::vector<Eigen::Vector3d> *vel_hist,
 							std::vector<Eigen::Vector3d> *acc_hist);

 	// void plot_traj_1D (const std::vector<double> &time_hist,
		//         	   const std::vector<Eigen::Vector3d> &pos_hist,
		// 			   const std::vector<Eigen::Vector3d> &vel_hist,
		// 			   const std::vector<Eigen::Vector3d> &acc_hist);
};

}  // namespace trapezoidal

#endif  // TRAPEZOIDAL_HELPER_H_
