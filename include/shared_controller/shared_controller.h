#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <atomic>
#include <memory>
#include <variant>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "dwal_planner/msg/cluster_group.hpp"
#include "dwal_planner/msg/path_cluster.hpp" 
#include "dwal_planner/msg/path.hpp"
#include "helper_fcns.h"
#include "dwal_intent_estimator.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>

class SharedController : public rclcpp::Node
{
public:
  explicit SharedController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void userCmdCallback( const geometry_msgs::msg::Twist::SharedPtr msg);
  void userCmdStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void publishIntentPlotAtClusterHorizonMeanPhi(const DWALClusterIntentEstimator::Result& intent,const dwal_planner::msg::ClusterGroup& clusters_msg);
  void clustersNearCallback(const dwal_planner::msg::ClusterGroup::SharedPtr msg);
  void clustersFarCallback(const dwal_planner::msg::ClusterGroup::SharedPtr msg);
  inline void publishCmd(const geometry_msgs::msg::TwistStamped& cmd);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  geometry_msgs::msg::TwistStamped MakeZeroTwist();

  // Helpers
  double scoreJ(double c_norm, double d_norm) const;

  // Vis
  nav_msgs::msg::Path shared_path;
  nav_msgs::msg::Path human_path;

  // Intent estimator
  std::unique_ptr<DWALClusterIntentEstimator> intent_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intent_plot_pub_;

  // Topics
  std::string cluster_topic_;
  std::string user_cmd_topic_;
  std::string output_cmd_topic_;
  std::string traj_viz_topic_;
  std::string traj_frame_id_;
  std::string odom_topic_;

  // Params
  double c_max_{255.0};
  double alpha_{0.5};
  double s_min_{0.2};
  double traj_horizon_s_{2.0};
  double traj_ds_{0.05};
  double Kmax_;  //from dwal
  double Kphi_;
  double level_; //from each cluster
  double phi_max_; //corresponding to Kmax and level
  bool use_stamped_twist_input_{true};
  bool use_stamped_twist_output_{true}; 
  double v_low_{0.25};   // hysteresis switch lower bound
  double v_high_{0.35};  // hysteresis switch upper bound
  std::string odom_frame_id_;

  // State
  bool have_user_cmd_{false};
  bool has_odom_{false};
  bool have_clusters_{false};

  geometry_msgs::msg::TwistStamped last_user_cmd_;
  dwal_planner::msg::ClusterGroup last_clusters_;
  std::string last_cluster_selected_;
  geometry_msgs::msg::Twist last_robot_vel_;

  // PS4 controller state
  std::atomic_bool r2_pressed_{false};
  int r2_button_index_{7};   // set via YAML; 7 is a common DS4 mapping but don’t assume

  std::shared_ptr<const dwal_planner::msg::ClusterGroup> near_ptr_{nullptr};
  std::shared_ptr<const dwal_planner::msg::ClusterGroup> far_ptr_{nullptr};

  // ROS I/O
  //rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr user_cmd_sub_;
  rclcpp::Subscription<dwal_planner::msg::ClusterGroup>::SharedPtr clusters_sub_far_;
  rclcpp::Subscription<dwal_planner::msg::ClusterGroup>::SharedPtr clusters_sub_near_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;


  using PubVariant = std::variant<
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr>;
  PubVariant fused_pub_;

  using SubVariant = std::variant<
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr,
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr>;
  SubVariant user_cmd_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_human_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_shared_publisher;
};

