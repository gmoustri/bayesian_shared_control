/*********************************************************************
 Mod of simple_trajectory_generator. Uses only DWA and exposes input space samples
 *********************************************************************/
#include <shared_controller/helper_fcns.h>  

#include <rclcpp/rclcpp.hpp>

int sgn(double val)
{
  return (int)(0.0 < val) - (val < 0.0);
}

double sat(double in, double lim_up, double lim_down)
{
  if (in > lim_up)
    return (lim_up);
  else if (in < lim_down)
    return (lim_down);
  else
    return in;
}

double rate_limiter(double d_value, double rate_lim, double dt)
{
  double a = d_value / dt;
  if (std::fabs(a) <= rate_lim)
    return d_value;
  else
    return rate_lim * dt * sgn(a);
}

namespace cluster_lib
{

double phi2curv(double phi, double R)
{
  if (std::fabs(phi) < M_PI_4)
    return 2 * std::sin(phi) / R;
  else
    return sgn(phi) / (R * std::cos(phi));
}

double curv2phi(double k, double R)
{
  if (std::fabs(k) < (M_SQRT2 / R))
    return std::asin(0.5 * R * k);
  else
    return sgn(k) * std::acos(1 / (std::fabs(k) * R));
}

double clusterChord(double Rcirc, double DPhi)
{
  return Rcirc * 2 * std::sin(0.5 * std::fabs(DPhi));
}

void generatePath(nav_msgs::msg::Path &path_out, double curv, double x0, double y0, double th0, double DS, double Rc2, std::string frame_id)
{
  // trajectory might be reused so we'll make sure to reset it
  path_out.header.stamp = rclcpp::Clock().now();
  path_out.header.frame_id = frame_id;
  path_out.poses.clear();

  double A, sincA, x, y, theta, dxLin, dyLin, Rp2;
  double dth = curv * DS;
  double S = 0;
  A = dth / 2;
  if (curv == 0)
    sincA = DS;
  else
    sincA = 2 * std::sin(A) / curv;

  geometry_msgs::msg::PoseStamped Posetmp;
  Posetmp.header.stamp = rclcpp::Clock().now();
  Posetmp.header.frame_id = "odom";
  
  x = x0;
  y = y0;
  theta = th0;

  dxLin = sgn(curv) * DS * -std::sin(th0); // cos(theta0+M_PI_2);
  dyLin = sgn(curv) * DS *  std::cos(th0); // sin(theta0+M_PI_2);

  // add initial point to the trajectory -- assume it's always free
  Posetmp.pose.position.x = x;
  Posetmp.pose.position.y = y;

  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    Posetmp.pose.orientation = tf2::toMsg(q);
  }

  path_out.poses.push_back(Posetmp);
  Rp2 = 0;

  // generate path
  while (Rp2 < Rc2)
  {
    S += DS;
    if (std::fabs(theta - th0) < M_PI_2)
    {
      x += sincA * std::cos(theta + A);
      y += sincA * std::sin(theta + A);
      theta += dth;
    }
    else
    {
      x += dxLin;
      y += dyLin;
    }
    Rp2 = (x - x0) * (x - x0) + (y - y0) * (y - y0);

    Posetmp.pose.position.x = x;
    Posetmp.pose.position.y = y;

    {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, theta);
      Posetmp.pose.orientation = tf2::toMsg(q);
    }

    path_out.poses.push_back(Posetmp);
  } // end path generation steps
}

} // namespace cluster_lib