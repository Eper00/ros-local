#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_example/utils.hpp"

void FirstScenario(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  moveit_visual_tools.prompt("Press 'Next' to start the first scenario");

  // Create collision object for the robot to avoid
  // Immediately Invoked Function Expression
  const auto [object, color] = []
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "box";
    object.header.frame_id = "world";

    // Define the size of the box
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.2;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.1;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.4;

    // Define the pose of the box
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.2;
    box_pose.position.z = 1.0;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(box_pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Define the color of the box
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0;
    color.a = 1.0;

    return std::make_pair(object, color);
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(object, color);

  moveit_visual_tools.prompt("Press 'Next' to plan around object");

  // Define target pose
  const auto target_pose = []
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = std::sin(Constants::PI / 2);
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 2);

    msg.position.x = 0.6;
    msg.position.y = 0.5;
    msg.position.z = 1.0;
    return msg;
  }();

  // Move to target pose
  MoveToPose(target_pose, move_group_interface, moveit_visual_tools, logger);
}
