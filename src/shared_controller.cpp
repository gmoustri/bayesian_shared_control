#include <shared_controller/shared_controller.h>

#include <algorithm>
#include <cmath>
#include <limits>

SharedController::SharedController(const rclcpp::NodeOptions &options)
    : rclcpp::Node("shared_controller", options)
{
  // Topics + frames
  cluster_topic_ = this->declare_parameter<std::string>("cluster_topic", "/dwal_planner/clusters_near");
  user_cmd_topic_ = this->declare_parameter<std::string>("user_cmd_topic", "/user_cmd");
  output_cmd_topic_ = this->declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");

  // Core params
  c_max_ = this->declare_parameter<double>("c_max", 255.0); // c_max
  alpha_ = this->declare_parameter<double>("alpha", 0.5);   // (0,1]
  s_min_ = this->declare_parameter<double>("s_min", 0.2);   // (0,1]
  Kphi_ = this->declare_parameter<double>("Kphi", 2.0);     // 2.0
  Kmax_ = this->declare_parameter<double>("Kmax", 2.0);     // 2.0
  use_stamped_twist_input_ = this->declare_parameter<bool>("use_stamped_twist_input", true);
  use_stamped_twist_output_ = this->declare_parameter<bool>("use_stamped_twist_output", true);
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
  r2_button_index_ = this->declare_parameter<int>("r2_button_index", 7);
  auto joy_topic   = this->declare_parameter<std::string>("joy_topic", "/joy");

  v_low_  = this->declare_parameter<double>("v_low", 0.25);
  v_high_ = this->declare_parameter<double>("v_high", 0.35);

  if (c_max_ <= 0.0)
      throw std::invalid_argument("SharedController: param 'c_max' must be > 0");
  if (alpha_ <= 0.0 || alpha_ > 1.0)
    throw std::invalid_argument("SharedController: param 'alpha' must be in (0, 1]");
  if (s_min_ <= 0.0 || s_min_ > 1.0)
    throw std::invalid_argument("SharedController: param 's_min' must be in (0, 1]");
  if (cluster_topic_.empty())
    throw std::invalid_argument("SharedController: param 'cluster_topic' must not be empty");
  if (user_cmd_topic_.empty())
    throw std::invalid_argument("SharedController: param 'user_cmd_topic' must not be empty");
  if (output_cmd_topic_.empty())
    throw std::invalid_argument("SharedController: param 'output_cmd_topic' must not be empty");
  if (Kphi_ <= 0)
    throw std::invalid_argument("Kphi: param must be > 0");
  if (Kmax_ <= 0)
    throw std::invalid_argument("Kmax: param must be > 0");
  if (v_low_ >= v_high_)
  {
    RCLCPP_WARN(get_logger(),
      "cluster_switch.v_low >= v_high. Adjusting to maintain hysteresis.");
    v_low_ = 0.25;
    v_high_ = 0.35;
  }
  // Intent estimator
  
  DWALClusterIntentEstimator::Params ip;
  ip.alpha = this->declare_parameter<double>("intent.alpha", 0.92);
  ip.beta = this->declare_parameter<double>("intent.beta", 8.0);
  ip.eta = this->declare_parameter<double>("intent.eta", 0.5);
  ip.sigma_min = this->declare_parameter<double>("intent.sigma_min", 0.03);
  ip.min_confidence = this->declare_parameter<double>("intent.min_confidence", 0.0);

  intent_ = std::make_unique<DWALClusterIntentEstimator>(ip);

  // Subscriptions & publisher

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic, rclcpp::QoS(10),
    [this](const sensor_msgs::msg::Joy::SharedPtr msg)
      {
        bool pressed = false;
        if (r2_button_index_ >= 0 && r2_button_index_ < (int)msg->buttons.size()) {
          pressed = (msg->buttons[r2_button_index_] != 0);
        }
        r2_pressed_.store(pressed, std::memory_order_relaxed);
      });


   odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(1),
      std::bind(&SharedController::odomCallback, this, std::placeholders::_1));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  clusters_sub_near_ = this->create_subscription<dwal_planner::msg::ClusterGroup>(
      "/dwal_planner/clusters_near",
      qos,
      std::bind(&SharedController::clustersNearCallback, this, std::placeholders::_1));

  clusters_sub_far_ = this->create_subscription<dwal_planner::msg::ClusterGroup>(
      "/dwal_planner/clusters_far",
      qos,
      std::bind(&SharedController::clustersFarCallback, this, std::placeholders::_1));

  intent_plot_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/shared_controller/intent_plot", rclcpp::QoS(10));

  //if user wants stamped input, subscribe to TwistStamped, else subscribe to Twist (for compatibility with different robot interfaces)
  if (use_stamped_twist_input_) {
      user_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        user_cmd_topic_, rclcpp::QoS(10),
        std::bind(&SharedController::userCmdStampedCallback, this, std::placeholders::_1));
  } else {
  user_cmd_sub_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
      user_cmd_topic_, rclcpp::QoS(10),
      std::bind(&SharedController::userCmdCallback, this, std::placeholders::_1));
  }

  //if  user wants stamped output, create publisher for TwistStamped, else for Twist (for compatibility with different robot interfaces)
  if (use_stamped_twist_output_) 
    fused_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_cmd_topic_, rclcpp::QoS(10));
  else 
    fused_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, rclcpp::QoS(10));
  
  path_human_publisher = this->create_publisher<nav_msgs::msg::Path>("/shared_controller/path_human", rclcpp::QoS(1));
  path_shared_publisher = this->create_publisher<nav_msgs::msg::Path>("/shared_controller/path_shared", rclcpp::QoS(1));

  RCLCPP_INFO(
      get_logger(),
      "\n SharedController up. \n Subscribing: clusters='%s \n user_cmd='%s' \n Publishing fused cmd to '%s'\n Kmax = %.2f \n Phi max (from Kmax) = %.2f rad (%.1f deg)",
      cluster_topic_.c_str(), user_cmd_topic_.c_str(), output_cmd_topic_.c_str(), Kmax_, phi_max_, phi_max_ * 180.0 / M_PI);
      
  RCLCPP_INFO(get_logger(), "Cluster hysteresis: v_low=%.2f, v_high=%.2f", v_low_, v_high_);
}

void SharedController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if(!has_odom_) {
    has_odom_ = true;
    odom_frame_id_= msg->header.frame_id;
  }

  last_robot_vel_ = msg->twist.twist;
}

void SharedController::clustersNearCallback(const dwal_planner::msg::ClusterGroup::SharedPtr msg)
{
  std::atomic_store(&near_ptr_, std::shared_ptr<const dwal_planner::msg::ClusterGroup>(msg));
}

void SharedController::clustersFarCallback(const dwal_planner::msg::ClusterGroup::SharedPtr msg)
{
  std::atomic_store(&far_ptr_, std::shared_ptr<const dwal_planner::msg::ClusterGroup>(msg));
}

geometry_msgs::msg::TwistStamped SharedController::MakeZeroTwist()
{
  geometry_msgs::msg::TwistStamped out;
  geometry_msgs::msg::Twist z;
  z.linear.x = 0.0;
  z.linear.y = 0.0;
  z.linear.z = 0.0;
  z.angular.x = 0.0;
  z.angular.y = 0.0;
  z.angular.z = 0.0;

  out.header.frame_id = odom_frame_id_;
  out.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  out.twist = z;
  return out;
}

double SharedController::scoreJ(double c_norm, double d_norm) const
{
  // J = (1-alpha)*c_norm + alpha*d_norm
  const double a = std::clamp(alpha_, 1e-6, 1.0);
  return (1.0 - a) * c_norm + a * d_norm;
}

void SharedController::publishIntentPlotAtClusterHorizonMeanPhi(
  const DWALClusterIntentEstimator::Result& intent,
  const dwal_planner::msg::ClusterGroup& clusters_msg)
{
  if (!intent_plot_pub_) return;

  const size_t K = clusters_msg.clusters.size();
  if (K == 0) return;

  visualization_msgs::msg::MarkerArray ma;
  const auto stamp = this->now();
  const std::string frame_id = odom_frame_id_; // e.g. "base_link"

  // Clear previous
  {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id;
    del.header.stamp = stamp;
    del.ns = "intent_horizon";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);
  }

  auto prob = [&](size_t i)->double {
    if (!intent.belief.empty() && intent.belief.size() == K)
      return intent.belief[i];

    return (intent.valid &&
            intent.cluster_index >= 0 &&
            static_cast<size_t>(intent.cluster_index) == i)
           ? std::clamp(intent.confidence, 0.0, 1.0)
           : 0.0;
  };

  const double bar_w = 0.14;
  const double bar_d = 0.14;
  const double bar_max_h = 0.50;

  for (size_t i = 0; i < K; ++i)
  {
    const auto& cl = clusters_msg.clusters[i];
    if (cl.paths.empty()) continue;

    // φ_mean = (φ_k + φ_l)/2
    const double phi_k = cl.paths.front().phi;
    const double phi_l = cl.paths.back().phi;
    const double phi_mean = 0.5 * (phi_k + phi_l);

    const double r = cl.r;

    /// Local point on the horizon (robot frame at pose0)
    const double xL = r * std::cos(phi_mean);
    const double yL = r * std::sin(phi_mean);

    // Transform to odom using pose0 = [x0,y0,th0]
    const double x0  = cl.pose0[0];
    const double y0  = cl.pose0[1];
    const double th0 = cl.pose0[2];

    const double c = std::cos(th0);
    const double s = std::sin(th0);

    const double xO = x0 + c * xL - s * yL;
    const double yO = y0 + s * xL + c * yL;


    const double p = std::clamp(prob(i), 0.0, 1.0);
    const double h = std::max(1e-3, p * bar_max_h);

    const bool is_sel =
      intent.valid &&
      intent.cluster_index >= 0 &&
      static_cast<size_t>(intent.cluster_index) == i;

    visualization_msgs::msg::Marker b;
    b.header.frame_id = frame_id;
    b.header.stamp = stamp;
    b.ns = "intent_horizon_bars";
    b.id = static_cast<int>(i + 1);
    b.type = visualization_msgs::msg::Marker::CUBE;
    b.action = visualization_msgs::msg::Marker::ADD;

    b.pose.position.x = xO;
    b.pose.position.y = yO;
    b.pose.position.z = 0.5 * h;
    b.pose.orientation.w = 1.0;

    b.scale.x = bar_w;
    b.scale.y = bar_d;
    b.scale.z = h;

    if (is_sel) { b.color.r=1.0f; b.color.g=1.0f; b.color.b=0.2f; b.color.a=0.9f; }
    else        { b.color.r=0.2f; b.color.g=0.6f; b.color.b=1.0f; b.color.a=0.6f; }

    b.lifetime = rclcpp::Duration::from_seconds(0.2);
    ma.markers.push_back(b);

    // Text label next to bar
    visualization_msgs::msg::Marker txt;
    txt.header.frame_id = frame_id;
    txt.header.stamp = stamp;
    txt.ns = "intent_horizon_text";
    txt.id = static_cast<int>(1000 + i);
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;

    txt.pose.position.x = xO ;   // small offset so it doesn't overlap bar
    txt.pose.position.y = yO;
    txt.pose.position.z = 0.51 * h; // on top of bar
    txt.pose.orientation.w = 1.0;

    txt.scale.z = 0.15;                // text height
    txt.color.r = 0.0f;
    txt.color.g = 0.0f;
    txt.color.b = 0.0f;
    txt.color.a = 1.0f;

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer),"p=%.2f",p);

    txt.text = buffer;
    txt.lifetime = rclcpp::Duration::from_seconds(0.2);

    ma.markers.push_back(txt);
  }

  intent_plot_pub_->publish(ma);
}

inline void SharedController::publishCmd(const geometry_msgs::msg::TwistStamped& cmd)
{
  std::visit([&](auto& pub) {
    using PubT = std::decay_t<decltype(pub)>;

    if constexpr (std::is_same_v<PubT, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>) {
      pub->publish(cmd);
    } else {
      pub->publish(cmd.twist);
    }
  }, fused_pub_);
}

// If user sends Twist (no stamp), convert to TwistStamped and call the main callback
void SharedController::userCmdCallback( const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.frame_id = odom_frame_id_;
  twist_msg->header.stamp = this->now();
  twist_msg->twist = *msg;
  userCmdStampedCallback(twist_msg);
}

// Main callback for user command (expects TwistStamped)
void SharedController::userCmdStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  
  if(!has_odom_) {
    RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Waiting for odometry.");
    return;
  }

  // Always update last user cmd
  last_user_cmd_ = *msg;
  have_user_cmd_ = true;

  // === Get user command ===
  const double v_user_raw = last_user_cmd_.twist.linear.x;
  const double wz_user_raw = last_user_cmd_.twist.angular.z;

  // ===== Switch cluster according to velocity ======
  auto near = std::atomic_load(&near_ptr_);
  auto far = std::atomic_load(&far_ptr_);

  std::shared_ptr<const dwal_planner::msg::ClusterGroup> chosen;

  // helper lambdas (optional)
  const bool near_ok = (near && !near->clusters.empty());
  const bool far_ok = (far && !far->clusters.empty());

  if (!near_ok && !far_ok)
  {
    // Default: if clusters are missing, pass through user command if R2 held, else output zero 
    if (r2_pressed_.load(std::memory_order_relaxed) && have_user_cmd_) {
      RCLCPP_WARN_SKIPFIRST(this->get_logger(),
        "No clusters received yet. R2 held -> pass-through user command.");
    } else {
      RCLCPP_WARN_SKIPFIRST(this->get_logger(),
        "No clusters received yet. R2 not held. Only rotation is allowed.");
        last_user_cmd_.twist.linear.x = 0.0; // zero linear velocity, keep angular for rotation
    }
    publishCmd(last_user_cmd_ );
    return;
  }

  std::string cluster_selected;
  if (!near_ok)
  {
    chosen = far; // only far valid
    cluster_selected = "far";
    }
  else if (!far_ok){
    chosen = near; // only near valid
    cluster_selected = "near";
  }
  else {             // both valid: switch based on robot velocity
    
     // both valid: hysteresis based on robot velocity
    if (last_cluster_selected_ == "near")
    {
      if (last_robot_vel_.linear.x > v_high_)
      {
        chosen = far;
        cluster_selected = "far";
      }
      else
      {
        chosen = near;
        cluster_selected = "near";
      }
    }
    else   // currently far (or uninitialized)
    {
      if (last_robot_vel_.linear.x < v_low_)
      {
        chosen = near;
        cluster_selected = "near";
      }
      else
      {
        chosen = far;
        cluster_selected = "far";
      }
    }
    //chosen = (last_robot_vel_.linear.x < 0.3) ? near : far;
    //cluster_selected = (last_robot_vel_.linear.x < 0.3) ? "near" : "far";
  }
 // Log only on change
  if (cluster_selected != last_cluster_selected_)
  {
    RCLCPP_INFO(this->get_logger(),
                "Selected cluster: %s",
                cluster_selected.c_str());

    last_cluster_selected_ = cluster_selected;
  }

  const auto &clusters_msg = *chosen;
  
  // update level to match selected cluster
  level_=clusters_msg.clusters[0].r; 

 // Update phi_max based on current level and Kmax  
  phi_max_ = cluster_lib::curv2phi(Kmax_, level_); 
  
  // convert w to phi. Clip phi to avoid extreme curvatures
  const double phi_user = std::clamp(Kphi_ * wz_user_raw, -phi_max_, phi_max_); 
  
  // convert phi to curvature for intent estimation and cluster selection
  const double k_user = cluster_lib::phi2curv(phi_user, level_); 

  // ============================================================
  // Visualize human path (for debugging / insight)
  // ============================================================

  const double x0 = clusters_msg.clusters[0].pose0[0];
  const double y0 = clusters_msg.clusters[0].pose0[1];
  const double th0 = clusters_msg.clusters[0].pose0[2];

  cluster_lib::generatePath(human_path, k_user, x0, y0, th0, 0.2, level_ * level_, odom_frame_id_);
  path_human_publisher->publish(human_path);

  if (v_user_raw < 0.0)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Negative velocity command received. Passing through user command.");
    publishCmd(last_user_cmd_);
    return;
  }

  // ============================================================
  // Bayesian Intent update (use phi_user as observation; it’s already clipped to [-phi_max, phi_max])
  // ============================================================

  const auto intent = intent_->update(phi_user, clusters_msg);

  if (!intent.valid || intent.cluster_index < 0)
  {
    RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Intent estimator could not select a cluster. Outputting zero.");
    publishCmd(MakeZeroTwist());
    return;
  }
  const auto &chosen_cluster = clusters_msg.clusters[static_cast<size_t>(intent.cluster_index)];

  double phi_near = chosen_cluster.paths.front().phi;
  double best = std::numeric_limits<double>::infinity();
  for (const auto &p : chosen_cluster.paths)
  {
    const double d = std::fabs(p.phi - phi_user);
    if (d < best)
    {
      best = d;
      phi_near = p.phi;
    }
  }
  publishIntentPlotAtClusterHorizonMeanPhi(intent, clusters_msg);

  // ============================================================
  // 1) Cluster selection by intent (curvature agreement)
  //    For each cluster: find k_near (member curvature closest to k_des)
  //    Then choose cluster minimizing |k_near - k_des|.
  // ============================================================

  // int best_cluster_idx = -1;
  // double best_cluster_mismatch = std::numeric_limits<double>::infinity();
  // double phi_near = 0.0;
  // double mismatch = std::numeric_limits<double>::infinity();

  //   for (size_t i = 0; i < clusters_msg.clusters.size(); ++i) {
  //   const auto & cl = clusters_msg.clusters[i];
  //   if (cl.paths.empty()) {
  //     continue;
  //   }

  //   for (const auto & p : cl.paths) {
  //     const double dphi = std::fabs(p.phi - phi_user);
  //     if (dphi < mismatch) {
  //       mismatch = dphi;
  //       phi_near = p.phi;
  //     }
  //   }

  //   // mismatch is |phi_near - phi_des|
  //   if (mismatch < best_cluster_mismatch) {
  //     best_cluster_mismatch = mismatch;
  //     best_cluster_idx = static_cast<int>(i);
  //   }
  // }

  // if (best_cluster_idx < 0)
  // {
  //   // No usable cluster -> zero output (per spec)
  //   RCLCPP_WARN_SKIPFIRST(this->get_logger(), "No usable cluster found. Outputting zero command.");
  //   fused_pub_->publish(MakeZeroTwist());
  //   return;
  // }

  // const auto & chosen_cluster = clusters_msg.clusters[static_cast<size_t>(best_cluster_idx)];

  // ============================================================
  // 2) Path scoring within selected cluster
  //    d_norm = |phi-phi_des|/phi_span
  //    J = (1-alpha)*c_norm + alpha*d_norm
  // ============================================================
  bool found = false;
  double best_J = std::numeric_limits<double>::infinity();
  double phi_star = 0.0;
  double c_star = 0.0;
  double phi_span = chosen_cluster.paths.back().phi - chosen_cluster.paths.front().phi;

  for (const auto &p : chosen_cluster.paths)
  {
    const double phi = p.phi;
    const double c = static_cast<double>(p.cost);

    const double dev = std::fabs(phi - phi_near);
    const double c_norm = std::clamp(c / c_max_, 0.0, 1.0); // normalize cost to [0,1]
    const double d_norm = std::clamp(dev / phi_span, 0.0, 1.0);
    const double J = scoreJ(c_norm, d_norm);

    if (J < best_J)
    {
      best_J = J;
      phi_star = phi;
      c_star = c;
      found = true;
    }
  }

  if (!found)
  {
    // No admissible after guard => output zero command (per spec)
    publishCmd(MakeZeroTwist());
    RCLCPP_WARN_SKIPFIRST(this->get_logger(), "No admissible path found after curvature guard. Outputting zero command.");
    return;
  }

  // ============================================================
  // 3) Velocity damping based on selected cost
  //    c_hat = c_star/c_max
  //    v_out = v_user * max(s_min, 1 - c_hat)
  //    omega_out = v_out * k_star
  // ============================================================
  const double c_hat = std::clamp(c_star/ c_max_, 0.0, 1.0); //normalized cost of selected path (clamped to [0,1])
  const double scale = std::max(std::clamp(s_min_, 1e-6, 1.0), 1.0 - c_hat);
  double k_star = cluster_lib::phi2curv(phi_star, level_);

  geometry_msgs::msg::TwistStamped out = last_user_cmd_;
  out.twist.linear.x = v_user_raw * scale;

  if (std::fabs(out.twist.linear.x) < 1e-3) {
    out.twist.angular.z = wz_user_raw; // if very low velocity, allow inplace rotation without damping (for better usability when stopped)
  } else {
    out.twist.angular.z = out.twist.linear.x * k_star;
  }

  publishCmd(out);

  cluster_lib::generatePath(shared_path, k_star, x0, y0, th0, 0.2, level_ * level_, odom_frame_id_);
  path_shared_publisher->publish(shared_path);
}
