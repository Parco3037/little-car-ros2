// Candidate for Humble 1.1.15. Commands return through controller_server;
// this library never publishes chassis velocity or bypasses the existing guard.
#include <limits>
#include <mutex>
#include <sstream>
#include <chrono>
#include "approach_policy.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace raicom_nav2_workspace {
inline double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

class SettledGoalChecker : public nav2_core::GoalChecker {
 public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      const std::string &name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {
    node_=parent.lock();name_=name;
    nav2_util::declare_parameter_if_not_declared(node_,name_+".xy_goal_tolerance",rclcpp::ParameterValue(.02));
    nav2_util::declare_parameter_if_not_declared(node_,name_+".yaw_goal_tolerance",rclcpp::ParameterValue(pi/18.));
    nav2_util::declare_parameter_if_not_declared(node_,name_+".stateful",rclcpp::ParameterValue(false));
  }
  void reset() override {arrival_.reset();}
  bool isGoalReached(const geometry_msgs::msg::Pose &pose,const geometry_msgs::msg::Pose &goal,
      const geometry_msgs::msg::Twist &velocity) override {
    if (node_->get_parameter(name_+".stateful").as_bool() ||
        std::abs(node_->get_parameter(name_+".xy_goal_tolerance").as_double()-.02)>1e-9 ||
        std::abs(node_->get_parameter(name_+".yaw_goal_tolerance").as_double()-pi/18.)>1e-9) {
      arrival_.reset();return false;
    }
    return arrival_.reached(std::hypot(pose.position.x-goal.position.x,pose.position.y-goal.position.y),
      wrap(tf2::getYaw(goal.orientation)-tf2::getYaw(pose.orientation)),
      std::hypot(velocity.linear.x,velocity.linear.y),velocity.angular.z,steady_seconds());
  }
  bool getTolerances(geometry_msgs::msg::Pose &pose,geometry_msgs::msg::Twist &velocity) override {
    const double invalid=std::numeric_limits<double>::lowest();
    pose.position.x=pose.position.y=.02;pose.position.z=invalid;
    pose.orientation.x=pose.orientation.y=0.;pose.orientation.z=std::sin(pi/36.);pose.orientation.w=std::cos(pi/36.);
    velocity.linear.x=velocity.linear.y=.01;velocity.linear.z=invalid;
    velocity.angular.x=velocity.angular.y=invalid;velocity.angular.z=.02;
    return true;
  }
 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;std::string name_;SettledArrival arrival_;
};

// Stateless replacement: leaving the XY window immediately restores translation.
// Read the same live checker parameters every cycle, rather than caching onInit.
class UnifiedRotateToGoalCritic : public dwb_core::TrajectoryCritic {
 public:
  bool prepare(const geometry_msgs::msg::Pose2D &pose,
      const nav_2d_msgs::msg::Twist2D &,
      const geometry_msgs::msg::Pose2D &goal, const nav_2d_msgs::msg::Path2D &) override {
    auto node = node_.lock();
    if (!node) return false;
    const double xy = node->get_parameter("general_goal_checker.xy_goal_tolerance").as_double();
    yaw_ = node->get_parameter("general_goal_checker.yaw_goal_tolerance").as_double();
    if (node->get_parameter("general_goal_checker.stateful").as_bool() ||
        std::abs(xy-.02)>1e-9 || std::abs(yaw_-pi/18.)>1e-9) return false;
    inside_ = std::hypot(goal.x-pose.x, goal.y-pose.y)<=xy;
    goal_yaw_ = goal.theta;
    return true;
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) override {
    if (!inside_) return 0.;
    if (trajectory.poses.empty()) throw dwb_core::IllegalTrajectoryException(name_, "empty trajectory");
    if (std::hypot(trajectory.velocity.x, trajectory.velocity.y)>.001)
      throw dwb_core::IllegalTrajectoryException(name_, "position reached: stop translation");
    return std::max(0., std::abs(wrap(goal_yaw_-trajectory.poses.back().theta))-yaw_);
  }
 private:
  bool inside_{false}; double yaw_{pi/18.}, goal_yaw_{0.};
};

// ThetaStar's grid endpoint can differ from the requested coordinates by more
// than the arrival tolerance. Preserve the exact requested endpoint for BOTH
// controller_server's goal checker and the controller; never relax acceptance.
class ExactGoalPlanner : public nav2_core::GlobalPlanner {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>("nav2_core", "nav2_core::GlobalPlanner");
    inner_ = loader_->createSharedInstance("nav2_theta_star_planner/ThetaStarPlanner");
    inner_->configure(parent, name, tf, costmap);
  }
  void activate() override {inner_->activate();}
  void deactivate() override {inner_->deactivate();}
  void cleanup() override {inner_->cleanup(); inner_.reset(); loader_.reset();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal) override {
    auto path = inner_->createPlan(start, goal);
    if (path.poses.empty()) return path;
    const auto &last = path.poses.back();
    if (path.header.frame_id != goal.header.frame_id ||
        std::hypot(last.pose.position.x-goal.pose.position.x,
                   last.pose.position.y-goal.pose.position.y)>.06)
      throw nav2_core::PlannerException("planner endpoint is not adjacent to exact goal");
    path.poses.push_back(goal);
    return path;
  }
 private:
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader_;
  nav2_core::GlobalPlanner::Ptr inner_;
};

class PhasedDwbController : public nav2_core::Controller {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_ = parent.lock(); name_ = name; tf_ = tf; costmap_ = costmap;
    if (!node_) throw nav2_core::PlannerException("missing controller node");
    for (const auto &entry : std::vector<std::pair<std::string,double>>{
        {"far_speed",.25},{"near_speed",.12},{"fine_speed",.03},{"turn_speed",.35},
        {"near_distance",.25},{"fine_distance",.06},{"validated_cap",.25},
        {"xy_goal_tolerance",.02}})
      nav2_util::declare_parameter_if_not_declared(node_, name_+"."+entry.first, rclcpp::ParameterValue(entry.second));
    inner_ = std::make_unique<dwb_core::DWBLocalPlanner>();
    inner_->configure(parent, name, tf, costmap);
  }
  void cleanup() override {inner_->cleanup(); plan_.poses.clear();braking_hold_.reset();}
  void activate() override {inner_->activate();}
  void deactivate() override {inner_->deactivate();}
  void setPlan(const nav_msgs::msg::Path &path) override {
    if (plan_.poses.empty() || path.poses.empty() || path.header.frame_id!=plan_.header.frame_id ||
        std::hypot(path.poses.back().pose.position.x-plan_.poses.back().pose.position.x,
                   path.poses.back().pose.position.y-plan_.poses.back().pose.position.y)>1e-6 ||
        std::abs(wrap(tf2::getYaw(path.poses.back().pose.orientation)-
                      tf2::getYaw(plan_.poses.back().pose.orientation)))>1e-6) {
      aligning_=true;braking_hold_.reset();
    }
    plan_ = path;
    if (plan_.header.frame_id.empty()) throw nav2_core::PlannerException("missing path frame");
    for (auto &p : plan_.poses) {
      if (p.header.frame_id.empty()) p.header.frame_id=plan_.header.frame_id;
      if (p.header.frame_id!=plan_.header.frame_id) throw nav2_core::PlannerException("mixed path frames");
    }
    inner_->setPlan(plan_);
  }
  void setSpeedLimit(const double &limit, const bool &percentage) override {
    external_limit_ = limit==0. ? 2.0 : percentage ? param("validated_cap")*limit/100. : limit;
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &input_pose, const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *checker) override {
    if (plan_.poses.empty()) throw nav2_core::PlannerException("empty plan");
    Limits limits;
    limits.far = param("far_speed"); limits.near = param("near_speed"); limits.fine = param("fine_speed");
    limits.near_distance = param("near_distance"); limits.fine_distance = param("fine_distance");
    limits.validated_cap = param("validated_cap");
    limits.turn = param("turn_speed");
    limits.xy = node_->get_parameter("general_goal_checker.xy_goal_tolerance").as_double();
    limits.yaw = node_->get_parameter("general_goal_checker.yaw_goal_tolerance").as_double();
    try {limits.validate();} catch (const std::exception &e) {throw nav2_core::PlannerException(e.what());}
    // Operator adjustment is bounded by the read-back guard and 2.0 m/s
    // software ceiling. Raising the setting is not physical speed acceptance.
    if (limits.validated_cap>2.0 || std::abs(param("max_vel_theta")-limits.turn)>1e-9 ||
        node_->get_parameter("general_goal_checker.stateful").as_bool() ||
        std::abs(param("xy_goal_tolerance")-limits.xy)>1e-9 || param("min_vel_x")<0.)
      throw nav2_core::PlannerException("approach/checker/forward envelope mismatch");
    limits.validated_cap = std::min(limits.validated_cap, external_limit_);
    if (!std::isfinite(limits.validated_cap) || limits.validated_cap<=0.)
      throw nav2_core::PlannerException("invalid external speed limit");
    const auto frame = costmap_->getGlobalFrameID();
    auto pose = transformed(input_pose, frame);
    auto goal = transformed(plan_.poses.back(), frame);
    double x = pose.pose.position.x, y = pose.pose.position.y, yaw = tf2::getYaw(pose.pose.orientation);
    double distance = std::hypot(goal.pose.position.x-x, goal.pose.position.y-y);
    double final_error = wrap(tf2::getYaw(goal.pose.orientation)-yaw);
    geometry_msgs::msg::PoseStamped aim = goal;
    if (distance>limits.near_distance) {
      const auto path_pose=transformed(input_pose,plan_.header.frame_id);
      const double path_x=path_pose.pose.position.x,path_y=path_pose.pose.position.y;
      std::size_t nearest = 0; double best = std::numeric_limits<double>::infinity();
      for (std::size_t i=0; i<plan_.poses.size(); ++i) {
        const auto &p=plan_.poses[i];
        double d = std::hypot(p.pose.position.x-path_x,p.pose.position.y-path_y);
        if (d<best) {best=d; nearest=i;}
      }
      for (std::size_t i=nearest; i<plan_.poses.size(); ++i) {
        aim=plan_.poses[i];
        if (std::hypot(aim.pose.position.x-path_x,aim.pose.position.y-path_y)>=.15) break;
      }
      aim=transformed(aim,frame);
    }
    double heading = wrap(std::atan2(aim.pose.position.y-y, aim.pose.position.x-x)-yaw);
    geometry_msgs::msg::TwistStamped result; result.header = pose.header;
    switch (stable_phase(distance,final_error,heading,limits,aligning_)) {
      case Phase::complete: break;
      case Phase::path_turn:
        result.twist.angular.z = turn_rate(heading,.08,limits); break;
      case Phase::final_yaw:
        result.twist.angular.z = turn_rate(final_error,limits.yaw,limits); break;
      case Phase::fine_position:
        result.twist.linear.x = limits.speed(distance)*std::max(0.,std::cos(heading));
        result.twist.angular.z = std::clamp(heading*1.5,-limits.turn,limits.turn); break;
      case Phase::tracking:
        // Humble DWB ignores a non-percentage limit equal to its base limit.
        // Restore the current base first so leaving a slow segment can accelerate.
        inner_->setSpeedLimit(0.,false);
        inner_->setSpeedLimit(limits.speed(distance),false);
        result = inner_->computeVelocityCommands(input_pose,velocity,checker); break;
    }
    if (!std::isfinite(result.twist.linear.x) || !std::isfinite(result.twist.angular.z) ||
        result.twist.linear.x<0. || std::abs(result.twist.linear.y)>1e-9)
      throw nav2_core::PlannerException("non-forward or nonfinite command");
    result.twist.linear.x = std::min(result.twist.linear.x,limits.speed(distance));
    result.twist.angular.z = std::clamp(result.twist.angular.z,-limits.turn,limits.turn);
    // Decelerate fully before an in-place turn. The downstream smoother still
    // enforces acceleration limits; the collision sweep includes current speed.
    if (result.twist.linear.x==0. && std::abs(velocity.linear.x)>.01) result.twist.angular.z=0.;
    constrain_sweep(x,y,yaw,result.twist,velocity,limits);
    return result;
  }
 private:
  double param(const std::string &key) const {return node_->get_parameter(name_+"."+key).as_double();}
  geometry_msgs::msg::PoseStamped transformed(geometry_msgs::msg::PoseStamped pose, const std::string &frame) {
    pose.header.stamp = builtin_interfaces::msg::Time();
    if (pose.header.frame_id==frame) return pose;
    try {return tf_->transform(pose,frame,tf2::durationFromSec(.1));}
    catch (const tf2::TransformException &e) {throw nav2_core::PlannerException(e.what());}
  }
  void constrain_sweep(double x,double y,double yaw,geometry_msgs::msg::Twist &cmd,
      const geometry_msgs::msg::Twist &actual,const Limits &limits) {
    auto *map = costmap_->getCostmap();
    if (!std::isfinite(actual.linear.x) || !std::isfinite(actual.angular.z))
      throw nav2_core::PlannerException("nonfinite measured velocity");
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision(map);
    const auto footprint = costmap_->getRobotFootprint();
    if (footprint.size()<3 || !costmap_->isCurrent()) throw nav2_core::PlannerException("stale costmap/footprint");
    const double current_cost=collision.footprintCostAtPose(x,y,yaw,footprint);
    if (current_cost<0. || current_cost>=nav2_costmap_2d::LETHAL_OBSTACLE)
      throw nav2_core::PlannerException("current body footprint blocked; no braking hold permitted");
    double radius=0.;
    for (const auto &point:footprint) radius=std::max(radius,std::hypot(point.x,point.y));
    std::string blocked;
    // Actual momentum cannot be made safe merely by requesting a slower speed.
    // It must pass independently before considering any reduced command.
    auto clear = [&](double v, double w, const char *kind) {
      double px=x,py=y,pa=yaw;
      const double horizon=limits.braking_horizon(v,w);
      // No footprint corner travels more than half a map cell per sample.
      const double dt=std::min(.02,.5*map->getResolution()/std::max(.01,std::abs(v)+radius*std::abs(w)));
      if (!std::isfinite(horizon) || horizon>10. || !std::isfinite(dt) || dt<.0001)
        throw nav2_core::PlannerException("braking sweep outside supported envelope");
      for (double t=0.; t<=horizon+dt; t+=dt) {
        double cost=collision.footprintCostAtPose(px,py,pa,footprint);
        if (cost<0. || cost>=nav2_costmap_2d::LETHAL_OBSTACLE) {
          std::ostringstream detail;
          detail << "footprint blocked: " << kind << " t=" << t << " cost=" << cost
                 << " x=" << px << " y=" << py << " yaw=" << pa << " v=" << v << " w=" << w;
          blocked=detail.str(); return false;
        }
        px+=std::cos(pa)*v*dt; py+=std::sin(pa)*v*dt; pa+=w*dt;
        if (t>=.20) {
          v=std::copysign(std::max(0.,std::abs(v)-limits.accel*dt),v);
          w=std::copysign(std::max(0.,std::abs(w)-limits.angular_accel*dt),w);
        }
      }
      return true;
    };
    auto hold_or_fail = [&]() {
      if (!braking_hold_.wait(steady_seconds(),limits.braking_horizon(actual.linear.x,actual.angular.z)))
        throw nav2_core::PlannerException("braking hold expired: "+blocked);
      cmd=geometry_msgs::msg::Twist();
      RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "Braking within current goal, zero command: %s",blocked.c_str());
    };
    if (!clear(actual.linear.x,actual.angular.z,"measured braking")) {
      hold_or_fail();return;
    }
    const double scale=safe_scale(cmd.linear.x,cmd.angular.z,
      [&](double v,double w) {return clear(v,w,"requested braking");});
    if (scale==0.) {hold_or_fail();return;}
    braking_hold_.reset();
    if (scale<1.) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
        "Obstacle braking: reduced command scale=%.3f; %s",scale,blocked.c_str());
      cmd.linear.x*=scale; cmd.angular.z*=scale;
    }
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_; std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::unique_ptr<dwb_core::DWBLocalPlanner> inner_;
  nav_msgs::msg::Path plan_; double external_limit_{2.0}; bool aligning_{true};BrakingHold braking_hold_;
};
}  // namespace raicom_nav2_workspace
PLUGINLIB_EXPORT_CLASS(raicom_nav2_workspace::PhasedDwbController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(raicom_nav2_workspace::ExactGoalPlanner, nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(raicom_nav2_workspace::UnifiedRotateToGoalCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(raicom_nav2_workspace::SettledGoalChecker, nav2_core::GoalChecker)
