#include <cmath>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_example/utils.hpp"
#include "moveit_example/second_scenario.hpp"

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "moveit_example",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  const auto logger = rclcpp::get_logger("moveit_example");

  // Create an executor so that the visual tools have access to the event system
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
   
  move_group_interface.setPlannerId("LIN");
  //pilz_industrial_motion_planner :"PTP", "LIN", "CIRC"
  //https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html
  //omlp "RRTConnectkConfigDefault", "RRTkConfigDefault", "PRMkConfigDefault", "KPIECEkConfigDefault", "ESTkConfigDefault", "STRIDEkConfigDefault", "RRTstarkConfigDefault", "PRMstarkConfigDefault", "BiTRRTkConfigDefault", "FMTkConfigDefault"
  //https://moveit.picknik.ai/humble/doc/examples/ompl_interface/ompl_interface_tutorial.html
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools(
      node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel());
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.trigger();

  RCLCPP_INFO(logger, "Planning frame: %s\tEnd-effector link: %s",
    move_group_interface.getPlanningFrame().c_str(), move_group_interface.getEndEffectorLink().c_str());

  // Setup the scene
  Setup();
  MoveToHome(move_group_interface, logger);



  // Second scenario
  SecondScenario(move_group_interface, moveit_visual_tools, logger);
  ClearScene(moveit_visual_tools);
  MoveToHome(move_group_interface, logger);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
