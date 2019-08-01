
#ifndef P4_HELPER_H_
#define P4_HELPER_H_

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "drone_planner/min_time.h"
#include "drone_planner/trapezoidal.h"
#include "drone_planner/geometry_functions.h"

#include "polynomial_solver.h"
#include "polynomial_sampler.h"

#include "drone_planner/helper.h"

namespace p4_helper {

namespace DimensionIdx {
static constexpr int X = 0;
static constexpr int Y = 1;
static constexpr int Z = 2;
}

namespace DerivativeIdx {
static constexpr int Position = 0;
static constexpr int Velocity = 1;
static constexpr int Acceleration = 2;
static constexpr int Jerk = 3;
static constexpr int Snap = 4;
static constexpr int Crackle = 5;
static constexpr int Pop = 6;
}

void set_pos_eq_constraint(const geometry_msgs::Point &pos, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints);

void set_vel_eq_constraint(const geometry_msgs::Vector3 &vel, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints);

void set_acc_eq_constraint(const geometry_msgs::Vector3 &acc, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints);

void set_max_vel(const double &max_vel, const uint &seg_idx,
	             std::vector<p4::SegmentInequalityBound> *ineq_constraints);

void set_max_acc(const double &max_acc, const uint &seg_idx,
	             std::vector<p4::SegmentInequalityBound> *ineq_constraints);

void set_max_jerk(const double &max_jerk, const uint &seg_idx,
	              std::vector<p4::SegmentInequalityBound> *ineq_constraints);

void setup_min_time_problem(const drone_planner::min_time::Request &req,
							const bool &is_straight,
			                std::vector<double> *times,
						    std::vector<p4::NodeEqualityBound> *node_eq,
						    std::vector<p4::SegmentInequalityBound> *segment_ineq,
						    p4::PolynomialSolver::Options *solver_options);

// void setup_min_acc_problem(const drone_planner::minAccXYWpPVA::Request &req,
// 			               std::vector<double> *times,
// 						   std::vector<p4::NodeEqualityBound> *node_eq,
// 						   std::vector<p4::SegmentInequalityBound> *segment_ineq,
// 						   p4::PolynomialSolver::Options *solver_options);

// Returns true if all waypoints are closely aligned
bool is_trajectory_straight(const std::vector<geometry_msgs::Point> &pts);

void solve_initial_min_time_trajectory(const std::vector<double> &init_time_guess,
	                            	   const std::vector<p4::NodeEqualityBound> &node_eq,
									   const std::vector<p4::SegmentInequalityBound> &segment_ineq,
									   const p4::PolynomialSolver::Options &solver_options,
									   const std::vector<p4::NodeInequalityBound> &node_ineq,
									   std::vector<double> *times,
									   p4::PolynomialPath *path);

// p4_ros::PolyPVA segment_pva_coeff_from_path(const p4::PolynomialPath &path,
// 	                                        const std::vector<double> &times,
// 	                                        const uint &seg_idx, const uint &dimension_idx);

void tucker_polynomials_to_coeff_matrix(
			const p4::PolynomialPath &path,
			const std::vector<double> &times,
			Eigen::VectorXd *segment_times, Eigen::MatrixXd *coeff_matrix);

}  // namespace p4_helper

#endif  // P4_HELPER_H_