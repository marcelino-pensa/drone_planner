#ifndef HELPER_H_
#define HELPER_H_

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"

#include "drone_planner/PVA_4d.h"


namespace helper {

size_t factorial(size_t n);

double saturate(const double &value, const double &min_val,
	            const double &max_val);

std::vector<float> eigen_to_stdvector(const Eigen::VectorXd &eig_vec);

Eigen::VectorXd stdvector_to_eigen(const std::vector<double> &vec);

std::vector<double> scale_std_vector(const std::vector<double> &vec, 
	                                 const double factor);

geometry_msgs::Point eigen_to_ros_point(const Eigen::Vector3d &eig_vec);

geometry_msgs::Point ros_point(const double &x, const double &y, const double &z);

geometry_msgs::Vector3 eigen_to_ros_vector(const Eigen::Vector3d &eig_vec);

geometry_msgs::Vector3 ros_vector3(const double &x, const double &y, const double &z);

geometry_msgs::PoseStamped set_pose(const geometry_msgs::Point &pos, const double &yaw);

drone_planner::PVA_4d constuct_PVA (const Eigen::Vector3d &pos,
			                        const Eigen::Vector3d &vel,
			                        const Eigen::Vector3d &acc,
			                        const double &yaw, const double &yaw_vel,
			                        const double &yaw_acc, const double &time);

std::vector<float> diff_coeff(const std::vector<float> &coeff_in);

void get_perpendicular_vectors(const Eigen::Vector3d &v, Eigen::Vector3d *v1, Eigen::Vector3d *v2);

Eigen::VectorXd time_to_segment_time(const std::vector<double> &times);

std::vector<double> segment_time_to_time(const Eigen::VectorXd &segment_times);

}  // namespace helper

#endif  // HELPER_H_