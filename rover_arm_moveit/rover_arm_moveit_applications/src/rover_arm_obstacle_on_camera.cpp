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
    ros::init(argc, argv, "obstacle_camera_detection");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Sleeping to load rviz
    sleep(15.0);

    moveit::planning_interface::MoveGroup group("arm");
    group.allowReplanning(true);
    group.setPlanningTime(20.0);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());


    group.setNamedTarget("home");
    moveit::planning_interface::MoveGroup::Plan my_plan; 
    bool success = group.plan(my_plan);
    if (success) 
    {
	group.execute(my_plan);
	sleep(5.0);
	//now we plan from this NON-SINGULAR config
	group.setStartStateToCurrentState();	
    }

    //==================================
    //=====Planning to a pose goal======
    //==================================
    geometry_msgs::Pose target_pose1;
    
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.708;
    target_pose1.position.y = -0.19;
    target_pose1.position.z = 0.178;

    group.setPoseTarget(target_pose1);

    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal for the eef) %s", success? "":"FAILED");
    sleep(20.0); 


    return 0;
}
