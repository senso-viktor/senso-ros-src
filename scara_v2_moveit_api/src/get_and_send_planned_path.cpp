
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
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include "moveit_msgs/DisplayTrajectory.h"


bool subs_ok = false;
bool start = false;

geometry_msgs::Pose pos_and_vel;
moveit_msgs::DisplayTrajectory trajectory1;


void displayPathCallback(const moveit_msgs::DisplayTrajectory dispTraj){

    subs_ok = true;
    //ROS_INFO("Move dispTraj");
    trajectory1 = dispTraj;
    //ROS_INFO("Move okay");

    //sleep(2);
}

void moveitModeCallback(const std_msgs::Bool startCommand){

    ROS_INFO("Command from SCARA menu");
    start = startCommand.data;
}



double checkAndModifyZaxis(double desiredZvalue){

    static double q=0.04;

    if (desiredZvalue > 0.04){
        desiredZvalue = 0.04;
    }else if (desiredZvalue < 0.00){
        desiredZvalue = 0.00;
    }

    return (-desiredZvalue + q);
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "get_planned_path");
    ros::NodeHandle n1,n2,n3,n4;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(10);
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    ROS_INFO("Joint values %f %f %f",current_joint_values[0], current_joint_values[1], current_joint_values[2]);
    sleep(2);

    ros::Publisher pose_pub = n1.advertise<geometry_msgs::Pose>("/planned_poses_and_velocities",1000);
    //ros::Publisher mode = n2.advertise<std_msgs::Byte>("/modeSelect",1000);
    ros::Subscriber trajectory_sub = n3.subscribe("/move_group/display_planned_path",1000,displayPathCallback);
    ros::Subscriber scaraMenu_sub = n4.subscribe("/moveitModeStart",1000,moveitModeCallback);

    std_msgs::Byte selectedMode;
    selectedMode.data = 6;


    int last_size = -5;
    int i =0;
    while (ros::ok()){


        if (subs_ok){
            ROS_INFO("subscribe to display trajectory OK!");
                //mode.publish(selectedMode);

               if (trajectory1.trajectory[0].joint_trajectory.points.size() != last_size){
                    last_size = trajectory1.trajectory[0].joint_trajectory.points.size();
                   ROS_ERROR("PLAN SIZE = %d",last_size);
                    i=0;
                }

                if (i< trajectory1.trajectory[0].joint_trajectory.points.size())
                {
                    //ROS_INFO("Sending message %d",i);
                    pos_and_vel.position.x = trajectory1.trajectory[0].joint_trajectory.points[i].positions[0];
                    pos_and_vel.position.y = trajectory1.trajectory[0].joint_trajectory.points[i].positions[1];
                    pos_and_vel.position.z = checkAndModifyZaxis(trajectory1.trajectory[0].joint_trajectory.points[last_size-1].positions[2]);
                    pos_and_vel.orientation.x = trajectory1.trajectory[0].joint_trajectory.points[i].velocities[0];
                    pos_and_vel.orientation.y = trajectory1.trajectory[0].joint_trajectory.points[i].velocities[1];
                    pos_and_vel.orientation.z = 0.0;
                    if (start){
                        pose_pub.publish(pos_and_vel);
                    }else{
                        ROS_INFO("data not published!");
                    }

                    ROS_WARN("message sent %d [of %d]!!",i,trajectory1.trajectory[0].joint_trajectory.points.size());
                    ROS_WARN("message go on!! [%f %f %f]",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
                    i++;
                }else{
                    pos_and_vel.position.x = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].positions[0];
                    pos_and_vel.position.y = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].positions[1];
                    pos_and_vel.position.z = checkAndModifyZaxis(trajectory1.trajectory[0].joint_trajectory.points[last_size-1].positions[2]);
                    pos_and_vel.orientation.x = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].velocities[0];
                    pos_and_vel.orientation.y = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].velocities[1];
                    pos_and_vel.orientation.z = trajectory1.trajectory[0].joint_trajectory.points[last_size-1].velocities[2];
                    if (start){
                        pose_pub.publish(pos_and_vel);
                    }else{
                        ROS_INFO("data not published!");
                    }

                    ROS_ERROR("message stay!![%f %f %f]",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
                    //ROS_INFO("%f %f %f", pos_and_vel.position.x,pos_and_vel.position.y,pos_and_vel.position.z);
                }
        }
//        else{
//            pos_and_vel.position.x = 0.0;
//            pos_and_vel.position.y = 0.0;
//            pos_and_vel.position.z = 0.0;
//            pos_and_vel.orientation.x = 0.0;
//            pos_and_vel.orientation.y = 0.0;
//            pos_and_vel.orientation.z = 0.0;
//            pose_pub.publish(pos_and_vel);
//            ROS_ERROR("message stay on 0!! [%f %f %f]",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
//        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}