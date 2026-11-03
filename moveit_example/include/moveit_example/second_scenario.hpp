#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "moveit_example/utils.hpp"

void SetupScene()
{
  const auto [objects, colors] = []
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects(3);

    // Place the machine tool in front of the robot
    // 20x20x20 cm cube with 2 cm walls, the side facing the robot (-x) is open
    objects[0].id = "machine";
    objects[0].header.frame_id = "world";
    objects[0].primitives.resize(5);
    objects[0].primitive_poses.resize(5);

    // Bottom plate
    objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[0].primitives[0].dimensions.resize(3);
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.2;
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
    objects[0].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.02;
    objects[0].primitive_poses[0].position.x = 0.6;
    objects[0].primitive_poses[0].position.y = 0.0;
    objects[0].primitive_poses[0].position.z = 0.56;
    objects[0].primitive_poses[0].orientation.w = 1.0;

    // Top plate
    objects[0].primitives[1].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[0].primitives[1].dimensions.resize(3);
    objects[0].primitives[1].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.2;
    objects[0].primitives[1].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
    objects[0].primitives[1].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.02;
    objects[0].primitive_poses[1].position.x = 0.6;
    objects[0].primitive_poses[1].position.y = 0.0;
    objects[0].primitive_poses[1].position.z = 0.74;
    objects[0].primitive_poses[1].orientation.w = 1.0;

    // Back wall
    objects[0].primitives[2].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[0].primitives[2].dimensions.resize(3);
    objects[0].primitives[2].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.02;
    objects[0].primitives[2].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.2;
    objects[0].primitives[2].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.16;
    objects[0].primitive_poses[2].position.x = 0.69;
    objects[0].primitive_poses[2].position.y = 0.0;
    objects[0].primitive_poses[2].position.z = 0.65;
    objects[0].primitive_poses[2].orientation.w = 1.0;

    // Right wall
    objects[0].primitives[3].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[0].primitives[3].dimensions.resize(3);
    objects[0].primitives[3].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.18;
    objects[0].primitives[3].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.02;
    objects[0].primitives[3].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.16;
    objects[0].primitive_poses[3].position.x = 0.59;
    objects[0].primitive_poses[3].position.y = -0.09;
    objects[0].primitive_poses[3].position.z = 0.65;
    objects[0].primitive_poses[3].orientation.w = 1.0;

    // Left wall
    objects[0].primitives[4].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[0].primitives[4].dimensions.resize(3);
    objects[0].primitives[4].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.18;
    objects[0].primitives[4].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.02;
    objects[0].primitives[4].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.16;
    objects[0].primitive_poses[4].position.x = 0.59;
    objects[0].primitive_poses[4].position.y = 0.09;
    objects[0].primitive_poses[4].position.z = 0.65;
    objects[0].primitive_poses[4].orientation.w = 1.0;

    objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Place the conveyor next to the robot, 30 cm lower than the machine tool
    objects[1].id = "conveyor";
    objects[1].header.frame_id = "world";
    objects[1].primitives.resize(1);
    objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[1].primitives[0].dimensions.resize(3);
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.8;
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.3;
    objects[1].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.25;
    objects[1].primitive_poses.resize(1);
    objects[1].primitive_poses[0].position.x = 0.75;
    objects[1].primitive_poses[0].position.y = 0.55;
    objects[1].primitive_poses[0].position.z = 0.125;
    objects[1].primitive_poses[0].orientation.w = 1.0;
    objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Place the workpiece in the middle of the machine tool
    objects[2].id = "workpiece";
    objects[2].header.frame_id = "world";
    objects[2].primitives.resize(1);
    objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    objects[2].primitives[0].dimensions.resize(3);
    objects[2].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.1;
    objects[2].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.1;
    objects[2].primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.1;
    objects[2].primitive_poses.resize(1);
    objects[2].primitive_poses[0].position.x = 0.6;
    objects[2].primitive_poses[0].position.y = 0.0;
    objects[2].primitive_poses[0].position.z = 0.65;
    objects[2].primitive_poses[0].orientation.w = 1.0;
    objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Specify the color of each object in the scene
    std::vector<moveit_msgs::msg::ObjectColor> colors(3);

    // Specify the machine tool's color
    colors[0].color.r = 0.0;
    colors[0].color.g = 0.0;
    colors[0].color.b = 1.0;
    colors[0].color.a = 0.5;

    // Specify the conveyor's color
    colors[1].color.r = 0.2;
    colors[1].color.g = 0.2;
    colors[1].color.b = 0.2;
    colors[1].color.a = 1.0;

    // Specify the workpiece's color
    colors[2].color.r = 1.0;
    colors[2].color.g = 0.8;
    colors[2].color.b = 0.0;
    colors[2].color.a = 1.0;

    return std::make_pair(objects, colors);
  }();

  // Add the collision objects to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(objects, colors);
}

static const geometry_msgs::msg::Point GetObjectPosition(const std::string object_id)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  return planning_scene_interface.getObjects({object_id})[object_id].pose.position;
}

void SecondScenario(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    moveit_visual_tools::MoveItVisualTools &moveit_visual_tools,
    const rclcpp::Logger &logger)
{
  moveit_visual_tools.prompt("Press 'Next' to start the second scenario");

  SetupScene();

  // TODO 1
  // Move the end-effector next to the workpiece through the open side of the machine tool
  // and make sure that it is facing into the machine (horizontally, towards +x)
  //
  // You have to:
  // - Find the position of the workpiece
  // - Create the pre_pick_up_pose in front of the opening and the pick_up_pose
  // - Move the robot to the specified poses
  // ...
  // Find the position of the workpiece
  const auto workpiece_pos = GetObjectPosition("workpiece");
  RCLCPP_INFO(logger, "Workpiece position: %f %f %f", workpiece_pos.x, workpiece_pos.y, workpiece_pos.z);

  // Touch the side of the workpiece facing the robot
  const auto pick_up_pose = [&workpiece_pos]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 4);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 4);

    msg.position.x = workpiece_pos.x - 0.05 - 1e-3;
    msg.position.y = workpiece_pos.y;
    msg.position.z = workpiece_pos.z;
    return msg;
  }();

  // Stop in front of the opening first, then move into the machine tool
  const auto pre_pick_up_pose = [&pick_up_pose]
  {
    geometry_msgs::msg::Pose msg = pick_up_pose;
    msg.position.x -= 0.2;
    return msg;
  }();
  MoveToPose(pre_pick_up_pose, move_group_interface, moveit_visual_tools, logger);

 
  MoveToPose(pick_up_pose, move_group_interface, moveit_visual_tools, logger);

  // TODO 2
  // Simulate picking up the workpiece by attaching it to the end-effector
  // ...
  move_group_interface.attachObject("workpiece");
  moveit_visual_tools.prompt("Workpiece attached to the end-effector. Press 'Next' to continue");


  MoveToPose(pre_pick_up_pose, move_group_interface, moveit_visual_tools, logger);


  const auto drop_off_position = GetObjectPosition("conveyor");
  RCLCPP_INFO(logger, "Conveyor position: %f %f %f", drop_off_position.x, drop_off_position.y, drop_off_position.z);

  // The workpiece is in front of the end-effector, so the end-effector stops
  // right before the -x end of the conveyor
  const auto drop_off_pose = [&drop_off_position]
  {
    geometry_msgs::msg::Pose msg;

    msg.orientation.x = 0.0;
    msg.orientation.y = std::sin(Constants::PI / 4);
    msg.orientation.z = 0.0;
    msg.orientation.w = std::cos(Constants::PI / 4);

    msg.position.x = drop_off_position.x - 0.4 - 1e-3;
    msg.position.y = drop_off_position.y;
    msg.position.z = drop_off_position.z + 0.125 + 0.05 + 1e-3;
    return msg;
  }();
  move_group_interface.setPlanningTime(15.0);
  MoveToPose(drop_off_pose, move_group_interface, moveit_visual_tools, logger);

  // TODO 5
  // Drop the workpiece onto the conveyor by detaching it from the robot
  // ...
  move_group_interface.detachObject("workpiece");
  moveit_visual_tools.prompt("Workpiece detached from the end-effector. Press 'Next' to continue");

  // Clear path constraints
  move_group_interface.clearPathConstraints();

  // Move to the home position
  MoveToHome(move_group_interface, moveit_visual_tools, logger);
}
