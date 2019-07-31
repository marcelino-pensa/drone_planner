#ifndef VISUALIZATION_FUNCTIONS_H_
#define VISUALIZATION_FUNCTIONS_H_

#include "ros/ros.h"
#include <Eigen/Dense>

// Ros message types
#include <visualization_msgs/Marker.h>
// #include "p4_ros/PVA.h"
#include "drone_planner/PVA_4d.h"

// Gnuplot for plotting results
// #include "gnuplot-iostream.h"

namespace visualization {


// Some colors for visualization markers
class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

class TrajPublisher {
 public:
 	TrajPublisher();
 	TrajPublisher(ros::NodeHandle *nh);
 	void VisualizePath(const Eigen::MatrixXd &polyCoeff,
 		                 const Eigen::VectorXd &time);
 	void VisualizePath(const std::vector<Eigen::Vector3d> pos_hist);
 	void VisualizeWaypoints(const Eigen::MatrixXd &path);
 	void PubPVA_Markers(const geometry_msgs::Point &pos,
 		                  const geometry_msgs::Vector3 &vel,
                      const geometry_msgs::Vector3 &acc);
 	void PubYaw_Marker(const geometry_msgs::Point &pos, 
                     const double &yaw);
 	// void PubRealTimeTraj(const std::vector<p4_ros::PVA> &pva_vec,
 	// 	                 const double &sampling_freq,
 	// 	                 const double &final_time);
 	void PubRealTimeTraj(const std::vector<drone_planner::PVA_4d> &pva_vec,
                       const double &sampling_freq,
                       const double &final_time);
 	Eigen::Vector3d getPosPoly(const Eigen::MatrixXd &polyCoeff, const int &k,
 					           const double &t, const uint &n_coeff);
 	// void plot_results_gnuplot(const std::vector<p4_ros::PVA> &pva_vec);

 private:
 	double vis_traj_width_, yaw_vec_length_;
 	ros::NodeHandle nh_;

 	// Visualization markers
	visualization_msgs::Marker vis_pos_, vis_vel_, vis_acc_, vis_yaw_;
	visualization_msgs::Marker traj_vis_;

	// ROS publishers
	ros::Publisher wp_traj_vis_pub_, wp_path_vis_pub_;
  ros::Publisher vis_pos_pub_, vis_vel_pub_, vis_acc_pub_, vis_yaw_pub_;
};


}  // namespace visualization

#endif  // VISUALIZATION_FUNCTIONS_H_