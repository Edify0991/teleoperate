/*An Aubo task
Write Focus with Aubo i5
by Guo Haoran
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Aubo_Write_FOCUS");
  ros::NodeHandle node_handle;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Define the planning group name
  static const std::string PLANNING_GROUP = "manipulator_i5";


  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPoseReferenceFrame("base_link");


  // Create a planning scene interface object
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // Create a robot model information object
  const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Create an object of the visualization class
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();


  // Load remote control tool
  visual_tools.loadRemoteControl();


  // Create text
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.2;
  visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
  // Text visualization takes effect
  visual_tools.trigger();


  // Get the coordinate system of the basic information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());


  // Get the end of the basic information
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  // Visual terminal prompt (blocking)  阻塞函数，等待next按钮
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  //***************************************************************************************************  Home Position
 
  //获取txt文件中的坐标值
  float init_coordinate[261][1];
  float coordinate[3][87];
  ifstream focusfile("//home//adan//coordinate//focus.txt");

  for(int i = 0; i < 261; i++)
  {
      focusfile >> init_coordinate[i][0];
  }

  for(int j = 0; j < 87; j++)
  {
      for(int i = 0; i < 3; i++)
      {
          coordinate[i][j] = init_coordinate[3 * j + i][0];
      }
  }

  focusfile.close();


  //设置初始位置，6个数值对应六轴的角度
  std::vector<double> home_position;
  home_position.push_back(0);
  home_position.push_back(0);
  home_position.push_back(0);
  home_position.push_back(0);
  home_position.push_back(0);
  home_position.push_back(0);


  std::vector<double> first_position;
  first_position.push_back(2.2);
  first_position.push_back(0);
  first_position.push_back(1.57);
  first_position.push_back(0);
  first_position.push_back(0);
  first_position.push_back(0);
  move_group.setJointValueTarget(first_position);//设置关节坐标目标
  move_group.move();    //plan+execute=move

 
 
  //开始设置路径值
  tf::Quaternion q;
  q.setRPY(3.14,0,0);       //设置rpy

  for(int i = 0; i < 87; i++)
  {
      geometry_msgs::Pose target_pose1;
      target_pose1.position.x = coordinate[0][i];
      target_pose1.position.y = coordinate[1][i];
      target_pose1.position.z = coordinate[2][i];
      target_pose1.orientation.x = q.x();
      target_pose1.orientation.y = q.y();
      target_pose1.orientation.z = q.z();
      target_pose1.orientation.w = q.w();

      move_group.setPoseTarget(target_pose1);//设置位置坐标

     // Call the planner for planning calculations Note: This is just planning
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");


      // visual planning path in Rviz
      visual_tools.deleteAllMarkers();
      visual_tools.publishAxisLabeled(target_pose1, "pose1");
      visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
      // Parameter 1 (trajectory_): path information
      // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();


      // Perform planning actions
      move_group.execute(my_plan);
  }
/*

  move_group.setJointValueTarget(first_position);//设置关节坐标目标
  move_group.move();    //plan+execute=move

  for(int i = 33; i < 87; i++)
  {
      geometry_msgs::Pose target_pose1;
      target_pose1.position.x = coordinate[0][i];
      target_pose1.position.y = coordinate[1][i];
      target_pose1.position.z = coordinate[2][i];
      target_pose1.orientation.x = q.x();
      target_pose1.orientation.y = q.y();
      target_pose1.orientation.z = q.z();
      target_pose1.orientation.w = q.w();

      move_group.setPoseTarget(target_pose1);//设置位置坐标

     // Call the planner for planning calculations Note: This is just planning
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");


      // visual planning path in Rviz
      visual_tools.deleteAllMarkers();
      visual_tools.publishAxisLabeled(target_pose1, "pose1");
      visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
      // Parameter 1 (trajectory_): path information
      // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();


      // Perform planning actions
      move_group.execute(my_plan);
  }
*/

  // Move to the home point position
  move_group.setJointValueTarget(home_position);
  move_group.move();

 
 
 
 
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");


  // END_TUTORIAL
  ros::shutdown();
  return 0;
}