#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace Constants
{
  const auto PI = std::acos(-1);
}

// Add a box primitive (size, center position) to a collision object
void AddBox(moveit_msgs::msg::CollisionObject &object,
            double size_x, double size_y, double size_z,
            double x, double y, double z)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size_x, size_y, size_z};

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
}

// The robot cell (all sizes in meters, in the "world" frame):
//  - machine tool: 20x20x20 cm cube, 2 cm walls, the side facing the robot (-x) is open
//  - table under the machine tool
//  - workpiece: 10x10x10 cm cube in the middle of the machine tool
//  - conveyor: its top is 30 cm lower than the bottom of the machine tool
void Setup()
{
  const auto [objects, colors] = []
  {
    // Machine tool position (center of its bottom)
    const double mt_x = 0.55, mt_y = 0.0, mt_z = 0.40;
    const double size = 0.20, wall = 0.02, part = 0.10, drop = 0.30;
    const double h = size / 2, zc = mt_z + h;

    std::vector<moveit_msgs::msg::CollisionObject> objects(4);
    for (auto &object : objects)
    {
      object.header.frame_id = "world";
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
    }

    // Machine tool: bottom, top, back wall (+x), side walls (+y, -y)
    objects[0].id = "machine_tool";
    AddBox(objects[0], size, size, wall, mt_x, mt_y, mt_z + wall / 2);
    AddBox(objects[0], size, size, wall, mt_x, mt_y, mt_z + size - wall / 2);
    AddBox(objects[0], wall, size, size - 2 * wall, mt_x + h - wall / 2, mt_y, zc);
    AddBox(objects[0], size, wall, size - 2 * wall, mt_x, mt_y + h - wall / 2, zc);
    AddBox(objects[0], size, wall, size - 2 * wall, mt_x, mt_y - h + wall / 2, zc);

    // Table under the machine tool
    objects[1].id = "table";
    AddBox(objects[1], size, size, mt_z, mt_x, mt_y, mt_z / 2);

    // Workpiece on the floor of the machine tool
    objects[2].id = "workpiece";
    AddBox(objects[2], part, part, part, mt_x, mt_y, mt_z + wall + part / 2);

    // Conveyor
    objects[3].id = "conveyor";
    AddBox(objects[3], 1.0, 0.30, 0.08, 0.0, -0.55, mt_z - drop - 0.04);

    // Specify the color of each object in the scene
    std::vector<moveit_msgs::msg::ObjectColor> colors(4);
    colors[0].color.r = 0.6; colors[0].color.g = 0.6; colors[0].color.b = 0.65; colors[0].color.a = 0.6;
    colors[1].color.r = 0.55; colors[1].color.g = 0.4; colors[1].color.b = 0.25; colors[1].color.a = 1.0;
    colors[2].color.r = 1.0; colors[2].color.g = 0.5; colors[2].color.b = 0.0; colors[2].color.a = 1.0;
    colors[3].color.r = 0.2; colors[3].color.g = 0.2; colors[3].color.b = 0.2; colors[3].color.a = 1.0;

    return std::make_pair(objects, colors);
  }();

  // Add the collision objects to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(objects, colors);
}

// Automatic mode: run without waiting for 'Next' (set from the auto_run parameter)
inline bool auto_run = false;

// Plan to the target set in move_group_interface, show the plan in RViz
// (title, path of the end-effector, target pose), wait for 'Next', then execute
void PlanAndExecute(
    const std::string &title,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  // Clear previous markers and show the title above the robot
  const auto text_pose = []
  {
    auto msg = Eigen::Isometry3d::Identity();
    msg.translation().z() = 1.2;
    return msg;
  }();
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.publishText(text_pose, title, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  moveit_visual_tools.trigger();

  // Create a plan to the target
  const auto [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    const auto ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (!success)
  {
    moveit_visual_tools.publishText(text_pose, title + ": planning failed!", rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "%s: planning failed!", title.c_str());
    return;
  }

  // Show the path and the target pose of the end-effector
  const auto target_pose = [&move_group_interface, &plan]
  {
    moveit::core::RobotState state(move_group_interface.getRobotModel());
    state.setVariablePositions(plan.trajectory_.joint_trajectory.joint_names,
                               plan.trajectory_.joint_trajectory.points.back().positions);
    state.update();
    return state.getGlobalLinkTransform(move_group_interface.getEndEffectorLink());
  }();
  moveit_visual_tools.publishTrajectoryLine(
      plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
  moveit_visual_tools.publishAxisLabeled(target_pose, "target");
  moveit_visual_tools.trigger();

  // Wait for 'Next' (or just a short pause in automatic mode), then execute
  if (auto_run)
  {
    RCLCPP_INFO(logger, "%s", title.c_str());
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  else
  {
    moveit_visual_tools.prompt("Press 'Next' to execute: " + title);
  }
  move_group_interface.execute(plan);
}

void MoveToPose(
    const std::string &title,
    const geometry_msgs::msg::Pose target_pose,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  move_group_interface.setPoseTarget(target_pose);
  PlanAndExecute(title, move_group_interface, moveit_visual_tools, logger);
}

void MoveToHome(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  move_group_interface.setJointValueTarget({
    0.0, -Constants::PI / 2, Constants::PI / 2, 0.0, Constants::PI / 2, 0.0});
  PlanAndExecute("Home", move_group_interface, moveit_visual_tools, logger);
}
