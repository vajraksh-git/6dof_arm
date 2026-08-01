#include <memory>
#include <chrono>
#include <iostream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/point.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // =====================================================================
  // THE FATAL FLAW FIX: Start the executor IMMEDIATELY in a background thread.
  // Now the node's "ears" are actually turned on and listening to RViz.
  // =====================================================================
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner_thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(node->get_logger(), "Connecting to Robot...");
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "manipulator");
  move_group->setPlanningTime(10.0);

  // NOW we wait 2 seconds. Because the spinner thread is running in the background,
  // it is actively downloading the joint states during this pause!
  rclcpp::sleep_for(std::chrono::seconds(2));

  geometry_msgs::msg::PoseStamped initial_pose = move_group->getCurrentPose();
  std::cout << "\n=========================================\n";
  std::cout << "--- REAL INITIAL POSE ---\n";
  std::cout << "X: " << initial_pose.pose.position.x << "\n";
  std::cout << "Y: " << initial_pose.pose.position.y << "\n";
  std::cout << "Z: " << initial_pose.pose.position.z << "\n";
  std::cout << "=========================================\n";

  // =====================================================================
  // COLLISION CHECKING DEMO: add a static box obstacle to the planning
  // scene so we can visibly prove the planner avoids it (or refuses to
  // plan through it). This uses PlanningSceneInterface, NOT the URDF,
  // so the robot description itself is never modified.
  // =====================================================================
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject ground;
  ground.header.frame_id = move_group->getPlanningFrame();
  ground.id = "ground_plane";

  shape_msgs::msg::SolidPrimitive ground_primitive;
  ground_primitive.type = ground_primitive.BOX;
  ground_primitive.dimensions = {4.0, 4.0, 0.02};  // wide, thin slab

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.x = 0.0;
  ground_pose.position.y = 0.0;
  ground_pose.position.z = -0.01;  // top surface sits right at z = 0

  ground.primitives.push_back(ground_primitive);
  ground.primitive_poses.push_back(ground_pose);
  ground.operation = ground.ADD;

  planning_scene_interface.applyCollisionObject(ground);
  RCLCPP_INFO(node->get_logger(), "Added ground plane to scene.");


  auto sub_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_cb_group;

  auto target_callback = [node, move_group](const geometry_msgs::msg::Point::SharedPtr msg) {
    RCLCPP_INFO(node->get_logger(), "Received Target -> X: %.2f | Y: %.2f | Z: %.2f", msg->x, msg->y, msg->z);

    geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();
    geometry_msgs::msg::Pose target_pose;

    target_pose.orientation = current_pose.pose.orientation;
    target_pose.position.x = msg->x;
    target_pose.position.y = msg->y;
    target_pose.position.z = msg->z;

    move_group->setPoseTarget(target_pose);
    move_group->setStartStateToCurrentState();

    RCLCPP_INFO(node->get_logger(), "Planning path... Check RViz!");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    if (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Plan successful! Executing motion...");
      auto exec_result = move_group->execute(my_plan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Motion complete! Ready for next target.");
      } else {
        RCLCPP_ERROR(node->get_logger(), "Execution failed with code: %d", exec_result.val);
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to plan trajectory. Coordinate might be out of reach or blocked by an obstacle.");
    }
  };

  auto sub = node->create_subscription<geometry_msgs::msg::Point>(
    "/target_coordinate", 10, target_callback, sub_options
  );

  RCLCPP_INFO(node->get_logger(), "Ready! Open a new terminal and publish to /target_coordinate");

  // Keep the main thread alive until you hit Ctrl+C
  spinner_thread.join();
  rclcpp::shutdown();
  return 0;
}