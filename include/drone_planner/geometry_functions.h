#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdlib.h>
#include "ros/ros.h"

namespace linear_algebra {
	//Line parameterized as l = p0 + t.vec, t belongs to (-inf,inf)
	struct Line_3d{
		Eigen::Vector3d p0;
		Eigen::Vector3d vec;
	};

	//Find line that goes through two points p1, p2
	Line_3d lineThroughPoints(const Eigen::Vector3d &p1,
					          const Eigen::Vector3d &p2);

	//Calculate the distance between one point and a line
	double distancePoint2Line(const Eigen::Vector3d point,
					          const Line_3d line);
}  // namespace linear_algebra