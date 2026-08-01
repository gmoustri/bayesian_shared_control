#ifndef DWAL_CLUSTER_LIB_H_
#define DWAL_CLUSTER_LIB_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>


double sat(double in, double limup, double limdown);
double rate_limiter(double d_value, double rate_lim, double dt);
int sgn(double val);

namespace cluster_lib
{

void generatePath(nav_msgs::msg::Path &PathOut, double curv, double x0, double y0, double th0, double DS, double Rc2, std::string frame_id = "odom");
double phi2curv(double phi, double Rcirc);
double curv2phi(double k, double Rcirc);
double clusterChord(double Rcirc, double DPhi);

} /* namespace cluster_lib */

#endif /* DWAL_CLUSTER_LIB_H_ */
