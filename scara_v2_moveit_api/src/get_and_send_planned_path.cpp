
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "math.h"
#include <tf/transform_listener.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <industrial_trajectory_filters/filter_base.h>
#include <industrial_trajectory_filters/uniform_sample_filter.h>
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "joint_limits_interface/joint_limits.h"
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include "moveit_msgs/DisplayTrajectory.h"

geometry_msgs::Pose pos_and_vel;
moveit_msgs::DisplayTrajectory trajectory1;
bool subs_ok = false;


void displayPathCallback(const moveit_msgs::DisplayTrajectory dispTraj){

    subs_ok = true;     //da s ato prerobit ze bude len subs a nebude dalsie porovnavaniee...
    ROS_INFO("Move dispTraj");
    trajectory1 = dispTraj;
    ROS_INFO("Move okay");

    sleep(2);
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "get_planned_path");
    ros::NodeHandle n1,n2,n3;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(15);
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    ROS_INFO("Joint values %f %f %f",current_joint_values[0], current_joint_values[1], current_joint_values[2]);
    sleep(2);

    ros::Publisher pose_pub = n1.advertise<geometry_msgs::Pose>("/planned_poses_and_velocities",1000);
    ros::Subscriber trajectory_sub = n3.subscribe("/move_group/display_planned_path",1000,displayPathCallback);



    int last_size = -5;
    int i =0;
    while (ros::ok()){


        if (subs_ok){
               if (trajectory1.trajectory[0].joint_trajectory.points.size() != last_size){
                    last_size = trajectory1.trajectory[0].joint_trajectory.points.size();
                    i=0;
                }
                if (i< trajectory1.trajectory[0].joint_trajectory.points.size())
                {
                    //ROS_INFO("Sending message %d",i);
                    pos_and_vel.position.x = trajectory1.trajectory[0].joint_trajectory.points[i].positions[0];
                    pos_and_vel.position.y = trajectory1.trajectory[0].joint_trajectory.points[i].positions[1];
                    pos_and_vel.position.z = trajectory1.trajectory[0].joint_trajectory.points[i].positions[2];
                    pos_and_vel.orientation.x = trajectory1.trajectory[0].joint_trajectory.points[i].velocities[0];
                    pos_and_vel.orientation.y = trajectory1.trajectory[0].joint_trajectory.points[i].velocities[1];
                    pos_and_vel.orientation.z = trajectory1.trajectory[0].joint_trajectory.points[i].velocities[2];
                    pose_pub.publish(pos_and_vel);
                    ROS_WARN("message sent %d [of %d]!!",i,trajectory1.trajectory[0].joint_trajectory.points.size());
                    //ROS_INFO("%f %f %f", pos_and_vel.position.x,pos_and_vel.position.y,pos_and_vel.position.z);
                    i++;
                }else{
                      //current_joint_values = move_group.getCurrentJointValues();
                    //ROS_INFO("Joint values %f %f %f",current_joint_values[0], current_joint_values[1], current_joint_values[2]);
                    //ROS_INFO("Sending message");
                    pos_and_vel.position.x = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].positions[0];
                    pos_and_vel.position.y = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].positions[1];
                    pos_and_vel.position.z = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].positions[2];
                    pos_and_vel.orientation.x = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].velocities[0];
                    pos_and_vel.orientation.y = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].velocities[1];
                    pos_and_vel.orientation.z = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].velocities[2];
                    pose_pub.publish(pos_and_vel);
                    ROS_ERROR("message stay!!");
                    //ROS_INFO("%f %f %f", pos_and_vel.position.x,pos_and_vel.position.y,pos_and_vel.position.z);
                }
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}