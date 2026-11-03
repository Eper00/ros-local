#include <cmath>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "kr6_cell/utils.hpp"

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "kr6_cell",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  const auto logger = rclcpp::get_logger("kr6_cell");

  // Create an executor so that the visual tools have access to the event system
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
  move_group_interface.setPlanningPipelineId("ompl");
  move_group_interface.setPlannerId("RRTConnectkConfigDefault");

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools(
      node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel());
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.trigger();

  RCLCPP_INFO(logger, "Planning frame: %s\tEnd-effector link: %s",
    move_group_interface.getPlanningFrame().c_str(), move_group_interface.getEndEffectorLink().c_str());

  // Automatic mode: ros2 run kr6_cell kr6_cell --ros-args -p auto_run:=true
  auto_run = node->get_parameter_or("auto_run", false);

  // Move to home first: in the zero position the arm is inside the machine tool
  MoveToHome(move_group_interface, moveit_visual_tools, logger);

  // Setup the scene
  Setup();

  // Move in front of the open side of the machine tool, tool pointing into it (+x)
  const auto machine_tool_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 4);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 4);
    msg.position.x = 0.38;
    msg.position.y = 0.0;
    msg.position.z = 0.47;
    return msg;
  }();
  MoveToPose("In front of the machine tool", machine_tool_pose, move_group_interface, moveit_visual_tools, logger);
  MoveToHome(move_group_interface, moveit_visual_tools, logger);

  // Move above the conveyor, tool pointing down
  const auto conveyor_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = std::sin(Constants::PI / 2);
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);
    msg.position.x = 0.0;
    msg.position.y = -0.55;
    msg.position.z = 0.35;
    return msg;
  }();
  MoveToPose("Above the conveyor", conveyor_pose, move_group_interface, moveit_visual_tools, logger);
  MoveToHome(move_group_interface, moveit_visual_tools, logger);

  // Clear the markers at the end
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.trigger();

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
