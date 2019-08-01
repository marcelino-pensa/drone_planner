#include "drone_planner/helper.h"

namespace helper {

size_t factorial(size_t n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

double saturate(const double &value, const double &min_val,
	            const double &max_val) {
	return std::max(std::min(value, max_val), min_val);
}

std::vector<float> eigen_to_stdvector(const Eigen::VectorXd &eig_vec) {
	std::vector<float> vec;
	for (uint i = 0; i < eig_vec.size(); i++) {
		vec.push_back(eig_vec(i));
	}
	return vec;
}

std::vector<double> scale_std_vector(const std::vector<double> &vec, 
	                                 const double factor) {
	std::vector<double> vec_out;
	for (uint i = 0; i < vec.size(); i++) {
		vec_out.push_back(vec[i]*factor);
	}
	return vec_out;
}

geometry_msgs::Point eigen_to_ros_point(const Eigen::Vector3d &eig_vec) {
	geometry_msgs::Point pt;
	pt.x = eig_vec[0];
	pt.y = eig_vec[1];
	pt.z = eig_vec[2];
	return pt;
}

geometry_msgs::Point ros_point(const double &x, const double &y, const double &z) {
	geometry_msgs::Point pt;
	pt.x = x;
	pt.y = y;
	pt.z = z;
	return pt;
}

geometry_msgs::Vector3 eigen_to_ros_vector(const Eigen::Vector3d &eig_vec) {
	geometry_msgs::Vector3 vec;
	vec.x = eig_vec[0];
	vec.y = eig_vec[1];
	vec.z = eig_vec[2];
	return vec;
}

geometry_msgs::Vector3 ros_vector3(const double &x, const double &y, const double &z) {
	geometry_msgs::Vector3 vec;
	vec.x = x;
	vec.y = y;
	vec.z = z;
	return vec;
}

geometry_msgs::PoseStamped set_pose(const geometry_msgs::Point &pos, const double &yaw) {
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "fcu";
	pose.pose.position = pos;
	pose.pose.orientation.w = cos(yaw/2.0);
	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = sin(yaw/2.0);
	return pose;
}

drone_planner::PVA_4d constuct_PVA (const Eigen::Vector3d &pos,
	                         const Eigen::Vector3d &vel,
	                         const Eigen::Vector3d &acc,
	                         const double &yaw, const double &yaw_vel,
	                         const double &yaw_acc, const double &time) {
	drone_planner::PVA_4d pva;
	pva.pos = helper::eigen_to_ros_point(pos);
	pva.vel = helper::eigen_to_ros_vector(vel);
	pva.acc = helper::eigen_to_ros_vector(acc);
	pva.yaw = yaw;
	pva.yaw_vel = yaw_vel;
	pva.yaw_acc = yaw_acc;
	pva.time = time;
	return pva;
}

std::vector<float> diff_coeff(const std::vector<float> &coeff_in) {
	uint coeff_size = coeff_in.size();
	std::vector<float> coeff_out;
	for (uint i = 0; i < coeff_size-1; i++) {
		coeff_out.push_back(coeff_in[i+1]);
	}
	coeff_out.push_back(0.0);

	return coeff_out;
}

void get_perpendicular_vectors(const Eigen::Vector3d &v, Eigen::Vector3d *v1, Eigen::Vector3d *v2) {
	std::srand((unsigned int) time(0));
	Eigen::Vector3d rdn = Eigen::Vector3d::Random().normalized();
	Eigen::Vector3d v_bar = v.normalized();

	*v1 = (rdn - (rdn.dot(v_bar)*v_bar)).normalized();
	*v2 = v1->cross(v_bar);
}

Eigen::VectorXd time_to_segment_time(const std::vector<double> &times) {
	const uint n_w = times.size();  // Number of waypoints
	Eigen::VectorXd segment_times = Eigen::VectorXd::Zero(n_w-1);
	for (uint i = 0; i < n_w - 1; i++) {
		segment_times[i] = times[i+1] - times[i];
	}
	return segment_times;
}

std::vector<double> segment_time_to_time(const Eigen::VectorXd &segment_times) {
	std::vector<double> times;
	double cur_time = 0.0;
	times.push_back(cur_time);
	for (uint i = 0; i < segment_times.size(); i++) {
		cur_time = cur_time + segment_times[i];
		times.push_back(cur_time);
	}
	return times;
}

}  // namespace helper
