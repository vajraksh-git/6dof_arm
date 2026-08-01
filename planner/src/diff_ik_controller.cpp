#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <Eigen/Dense>

using namespace std::chrono_literals;

// =============================================================================
// Differential IK controller.
//
// Design decisions (documented here so they can be copied into the README):
//
// 1. Jacobian pseudo-inverse, not full re-planning per tick.
//    OMPL-based planning (Stage 1) is discrete and takes tens of ms to
//    seconds -- too slow for a responsive teleop loop. Differential IK
//    instead maps a desired *instantaneous* Cartesian velocity directly to
//    joint velocities using the manipulator Jacobian, recomputed every tick
//    from the live joint state. This is standard resolved-rate control.
//
// 2. Damped least squares (DLS), not a plain pseudo-inverse.
//    Near singular configurations the Jacobian becomes ill-conditioned and
//    a plain pseudo-inverse produces huge joint velocity spikes. DLS adds a
//    damping term lambda that trades a small amount of tracking accuracy
//    for bounded, stable output near singularities:
//        q_dot = J^T (J J^T + lambda^2 I)^-1 * x_dot
//
// 3. Per-joint velocity clamping.
//    Limits are read from the robot model (URDF/SRDF), never hardcoded,
//    per the "URDF-agnostic" project requirement.
//
// 4. Streamed JointTrajectory with a short time_from_start, not raw
//    velocity commands.
//    The configured controller is a JointTrajectoryController (position +
//    velocity interface), which has no standalone "velocity command" mode.
//    Publishing a single-point trajectory with a short horizon (e.g. 50ms)
//    every tick is the standard way to get continuous, low-latency motion
//    out of that controller type without switching controllers.
// =============================================================================

class DiffIKController : public rclcpp::Node
{
public:
  DiffIKController()
  : Node("diff_ik_controller")
  {
    declare_parameter("planning_group", "manipulator");
    declare_parameter("control_rate_hz", 50.0);
    declare_parameter("damping_lambda", 0.05);
    declare_parameter("controller_topic",
      "/manipulator_controller/joint_trajectory");

    planning_group_ = get_parameter("planning_group").as_string();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    lambda_ = get_parameter("damping_lambda").as_double();
    std::string controller_topic = get_parameter("controller_topic").as_string();

    dt_ = 1.0 / control_rate_hz_;

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cartesian_velocity_cmd", 10,
      std::bind(&DiffIKController::twistCallback, this, std::placeholders::_1));

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_topic, 10);
  }

  // Separate init() because it needs shared_from_this(), unavailable in ctor.
  void init()
  {
    robot_model_loader::RobotModelLoader::Options opts("robot_description");
    model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), opts);
    robot_model_ = model_loader_->getModel();

    if (!robot_model_) {
      RCLCPP_FATAL(get_logger(), "Failed to load robot model.");
      return;
    }

    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
    if (!joint_model_group_) {
      RCLCPP_FATAL(get_logger(), "Planning group '%s' not found.", planning_group_.c_str());
      return;
    }

    joint_names_ = joint_model_group_->getVariableNames();

    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");
    psm_->startStateMonitor();
    psm_->startSceneMonitor();

    // Wait for the first real joint state so we don't start from a garbage
    // default configuration.
    if (!psm_->getStateMonitor()->waitForCurrentState(this->now(), 5.0)) {
      RCLCPP_WARN(get_logger(),
        "Timed out waiting for initial joint states -- check /joint_states.");
    }

    current_positions_ = getCurrentJointPositions();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&DiffIKController::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Differential IK controller ready at %.1f Hz.", control_rate_hz_);
  }

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_twist_ = *msg;
    last_twist_time_ = now();
  }

  std::vector<double> getCurrentJointPositions()
  {
    moveit::core::RobotStatePtr state = psm_->getStateMonitor()->getCurrentState();
    std::vector<double> positions;
    state->copyJointGroupPositions(joint_model_group_, positions);
    return positions;
  }

  void controlLoop()
  {
    // Safety: if no twist has arrived recently, treat as zero. This is a
    // second layer of "stop immediately" protection beyond the publisher
    // side, in case the teleop node dies or the link drops.
    geometry_msgs::msg::Twist cmd = latest_twist_;
    if ((now() - last_twist_time_).seconds() > 0.5) {
      cmd = geometry_msgs::msg::Twist();
    }

    Eigen::VectorXd x_dot(6);
    x_dot << cmd.linear.x, cmd.linear.y, cmd.linear.z,
              cmd.angular.x, cmd.angular.y, cmd.angular.z;

    if (x_dot.norm() < 1e-6) {
      // Nothing commanded -- don't drift, don't republish stale points.
      return;
    }

    moveit::core::RobotStatePtr state = psm_->getStateMonitor()->getCurrentState();
    state->setJointGroupPositions(joint_model_group_, current_positions_);
    state->update();

    Eigen::MatrixXd jacobian;
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    if (!state->getJacobian(
          joint_model_group_,
          state->getLinkModel(joint_model_group_->getLinkModelNames().back()),
          reference_point, jacobian)) {
      RCLCPP_ERROR(get_logger(), "Failed to compute Jacobian.");
      return;
    }

    // Damped least squares pseudo-inverse.
    int rows = jacobian.rows();
    Eigen::MatrixXd JJt = jacobian * jacobian.transpose();
    Eigen::MatrixXd damped = JJt + (lambda_ * lambda_) * Eigen::MatrixXd::Identity(rows, rows);
    Eigen::VectorXd q_dot = jacobian.transpose() * damped.ldlt().solve(x_dot);

    clampToVelocityLimits(q_dot);

    for (size_t i = 0; i < current_positions_.size(); ++i) {
      current_positions_[i] += q_dot(i) * dt_;
    }

    publishTrajectoryPoint(q_dot);
  }

  void clampToVelocityLimits(Eigen::VectorXd & q_dot)
  {
    const auto & bounds = joint_model_group_->getActiveJointModelsBounds();
    for (size_t i = 0; i < bounds.size(); ++i) {
      double limit = (*bounds[i])[0].max_velocity_;
      if (limit > 0.0) {
        q_dot(i) = std::clamp(q_dot(i), -limit, limit);
      }
    }
  }

  void publishTrajectoryPoint(const Eigen::VectorXd & q_dot)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = current_positions_;
    point.velocities.assign(q_dot.data(), q_dot.data() + q_dot.size());

    // Short horizon so the controller treats this as "move there very
    // soon" -- effectively a continuous velocity stream when republished
    // every control-loop tick.
    double horizon = dt_ * 2.0;
    point.time_from_start = rclcpp::Duration::from_seconds(horizon);

    traj.points.push_back(point);
    traj_pub_->publish(traj);
  }

  std::string planning_group_;
  double control_rate_hz_;
  double lambda_;
  double dt_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  robot_model_loader::RobotModelLoaderPtr model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup * joint_model_group_ = nullptr;
  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  geometry_msgs::msg::Twist latest_twist_;
  rclcpp::Time last_twist_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiffIKController>();
  node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}