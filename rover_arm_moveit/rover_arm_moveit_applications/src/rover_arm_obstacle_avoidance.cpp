/*
BSD 2-Clause License

Copyright (c) 2016, Alessandro Santoni
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial_arm");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Loading RViz
    sleep(15.0);

    moveit::planning_interface::MoveGroup group("arm");
    group.allowReplanning(true);

    //The planning scene interface allows us to deal with the "world" of RViz
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    //Printing out reference frames
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    //I make use of the home position defined in the SRDF
    group.setNamedTarget("home");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    if (success) 
    {
	group.execute(my_plan);
	sleep(5.0);
    }

    //now we plan from this NON-SINGULAR position
    group.setStartStateToCurrentState();

    //==================================
    //=====Planning to a pose goal======
    //==================================
    
    geometry_msgs::Pose target_pose1;
    
    //Here i define the message
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.132;
    target_pose1.position.y = 0.545;
    target_pose1.position.z = 0.647;

    //Now i give this setpoint to my group object
    group.setPoseTarget(target_pose1);
    
    //Let's call the planner to compute and visualize this plan
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal for the eef) %s", success? "":"FAILED");
    sleep(10.0); 

  
    //Introduction of objects to see how trajectory changes
    //I define a collision object message first
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.header.frame_id = group.getPlanningFrame();
    collision_object1.id = "obstacle1";

    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.3;
    primitive1.dimensions[1] = 0.1;
    primitive1.dimensions[2] = 0.3;

    //Now i'm placing the box relatively to frame_id selected above
    geometry_msgs::Pose obstacle1_pose;
    obstacle1_pose.orientation.w = 1.0;
    obstacle1_pose.position.x = 0.0;
    obstacle1_pose.position.y = 0.3;
    obstacle1_pose.position.z = 0.75;

    collision_object1.primitives.push_back(primitive1);
    collision_object1.primitive_poses.push_back(obstacle1_pose);
    collision_object1.operation = collision_object1.ADD;

    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = group.getPlanningFrame();
    collision_object2.id = "obstacle2";

    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 0.3;
    primitive2.dimensions[1] = 0.3;
    primitive2.dimensions[2] = 0.1;

    geometry_msgs::Pose obstacle2_pose;
    obstacle2_pose.orientation.w = 1.0;
    obstacle2_pose.position.x = 0.0;
    obstacle2_pose.position.y = 0.5;
    obstacle2_pose.position.z = 0.2;
    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(obstacle2_pose);
    collision_object2.operation = collision_object2.ADD;


    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object1);
    collision_objects.push_back(collision_object2);


    //Let's now effectively add the objects into the world
    ROS_INFO("Obstacles spawn in the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    //Sleep to see the object in rviz
    sleep(5.0);

    //We want to increase allotted time for planning when obstacles are present
    group.setPlanningTime(10.0);

    //By giving a pose setpoint, the arm will avoid obstacles
    group.setPoseTarget(target_pose1);
    success = group.plan(my_plan);

    ROS_INFO("Visualizing same target pose avoiding obstacles...%s", success ? "" : "FAILED");

    sleep(20.0);
    return 0;
}
