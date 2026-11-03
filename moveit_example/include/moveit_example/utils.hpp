#pragma once

#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace Constants
{
  const auto PI = std::acos(-1);
}

void Setup()
{
  const auto [object, color] = []
  {
    moveit_msgs::msg::CollisionObject object;

    // Place plane
    object.id = "ground";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions.resize(3);
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 20;
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 20;
    object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.01;
    object.primitive_poses.resize(1);
    object.primitive_poses[0].position.x = 0.0;
    object.primitive_poses[0].position.y = 0.0;
    object.primitive_poses[0].position.z = -0.01;
    object.primitive_poses[0].orientation.w = 1.0;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Specify the plane's color
    std_msgs::msg::ColorRGBA color;
    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.5;
    color.a = 1.0;

    return std::make_pair(object, color);
  }();

  // Add the ground to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(object, color);
}

void MoveToPose(
    const geometry_msgs::msg::Pose target_pose,
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  // Clear previous markers
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.trigger();

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  const auto [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    const auto ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    moveit_visual_tools.publishTrajectoryLine(
        plan.trajectory_, move_group_interface.getRobotModel()->getJointModelGroup("manipulator"));
    moveit_visual_tools.trigger();
    moveit_visual_tools.prompt("Press 'Next' to continue");
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
}

void MoveToHome(moveit::planning_interface::MoveGroupInterface &move_group_interface, const rclcpp::Logger &logger)
{
  move_group_interface.setJointValueTarget({
    0.0, -Constants::PI / 2, Constants::PI / 2, 0.0, Constants::PI / 2, 0.0});

  // Create a plan to that target joint values
  const auto [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    const auto ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to plan to the home position!");
  }
}

void MoveToHome(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  // Clear previous markers
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.trigger();

  // Move to home
  MoveToHome(move_group_interface, logger);
}

void ClearScene(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools)
{
  moveit_visual_tools.prompt("Press 'Next' to clear the scene");
  const auto objects_to_remove = []
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects(0);
    for (const auto name : {"box", "table1", "table2", "cylinder", "machine", "conveyor", "workpiece"})
    {
      moveit_msgs::msg::CollisionObject object;
      object.id = name;
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      objects.push_back(object);
    }
    return objects;
  }();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(objects_to_remove);
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.trigger();
}
