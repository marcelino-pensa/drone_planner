#include "drone_planner/geometry_functions.h"

namespace linear_algebra {
	
	//3d_Line line that goes through two points p1, p2 in 2D
	Line_3d lineThroughPoints(const Eigen::Vector3d &p1,
					          const Eigen::Vector3d &p2) {
	// 	Eigen::Vector3d x1, x2;
	// 	x1 << p1(0), p1(1), 1;
	// 	x2 << p2(0), p2(1), 1;

	// 	return x1.cross(x2);
		Line_3d line;
		line.p0 = p1;
		line.vec = p1 - p2;

		return line;
	}

	//Calculate the distance between one point and a line in 2D
	double distancePoint2Line(const Eigen::Vector3d point,
					          const Line_3d line) {
		// const double x = point(0);
		// const double y = point(1);
		// const double a = line(0);
		// const double b = line(1);
		// const double c = line(2);

		// // ROS_INFO("x: %f\ty: %f\ta: %f\tb: %f\tc: %f",x,y,a,b,c);

		// //Algorithm obtained from 
		// //https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		// // ROS_INFO("dist: %f", std::abs(a*x + b*y + c));
		// return std::abs(a*x + b*y + c)/sqrt(pow(a,2) + pow(b,2));

		//Two points on the line
		const Eigen::Vector3d x1 = line.p0;
		const Eigen::Vector3d x2 = line.p0 + line.vec;

		//Optimal t is the t at which the point is closest to the line
		// t_opt = (x1-x2)'*(x1-p)/norm(x1-x2)^2;
		const double gain = 1.0/(pow((x1-x2).norm(),2.0));
		const double t_opt = gain*(x1-x2).transpose()*(x1-point);

		//p_opt is the closest point between the point and the line
		const Eigen::Vector3d p_opt = x1 + t_opt*line.vec;

		return (point - p_opt).norm();
	}

}  // namespace linear_algebra