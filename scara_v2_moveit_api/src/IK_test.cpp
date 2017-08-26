//
// Created by viktordluhos on 14/08/17.
//

//Adding followjointtrajectorygoal

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
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>


using namespace std;

std::vector<double> joint_positions(3);
std::vector<double> link_length(2);
double x_offset, y_offset, z_offset;
geometry_msgs::Point point;
bool executionOK = false;

geometry_msgs::Point getPoseFromTF(std::string source, std::string target) {

    geometry_msgs::Point point;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform(source, target, ros::Time(0), ros::Duration(1));
        listener.lookupTransform(source, target, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_WARN("OSM planner: %s. Can't update pose from TF, for that will be use the latest source point.",
                 ex.what());
    }
    tf::pointTFToMsg(transform.getOrigin(), point);
    return point;

}
void getOffsets(){

    //    Default values
//    link_length[0] = 0.24942;
//    link_length[1] = 0.24304;
//    offset_x = 0.212;
//    offset_y = 0.58;
//    offset_z = 1.01962;
    geometry_msgs::Point scaraLink1, scaraLink2, scaraBase;

    point = getPoseFromTF("world","BaseBox");
    x_offset = x_offset + point.x;
    y_offset = y_offset + point.y;
    point = getPoseFromTF("BaseBox","ScaraBase");
    x_offset = x_offset + point.x;
    y_offset = y_offset + point.y;
    point = getPoseFromTF("world","tool0");
    z_offset = point.z;

    scaraBase = getPoseFromTF("world","ScaraLink1");
    scaraLink1 = getPoseFromTF("world","ScaraLink2");
    scaraLink2 = getPoseFromTF("world","GripperBase");
    link_length[0] = sqrt (pow(scaraLink1.x-scaraBase.x,2.0) + pow(scaraLink1.y-scaraBase.y,2.0) );
    link_length[1] = sqrt (pow(scaraLink2.x-scaraLink1.x,2.0) + pow(scaraLink2.y-scaraLink1.y,2.0) );
    ROS_INFO("final offsets: x_offset=%f y_offset=%f z_offset=%f",x_offset, y_offset, z_offset);
    ROS_INFO("final arm lenths: arm1=%f arm2=%f",link_length[0], link_length[1]);
    sleep(2);

}

bool countIK(double x, double y, double z,  int mode){

    ROS_INFO("input numbers %f %f %f",x,y,z);
    x = x - x_offset;
    y = y - y_offset;
    z = z - z_offset;
    //ROS_INFO("input numbers - offset = %f %f %f",x,y,z);

    double c2 = (pow(x,2.0) + pow(y,2.0) - pow(link_length[0],2.0) - pow(link_length[1],2.0))/(2*link_length[0]*link_length[1]);
    if ((1-c2) > 0.0 ){
        if (mode == 1) {
            joint_positions[1] = atan2(-sqrt(1 - c2), c2);
        }else{
            joint_positions[1] = atan2(sqrt(1 - c2), c2);
        }
        joint_positions[0] =  atan2(y,x) - atan2(link_length[1]*sin(joint_positions[1]),link_length[0] + link_length[1]*cos(joint_positions[1]));
        joint_positions[1] = - joint_positions[1] ; //otocenie kvoli opacnej rotacii klbu
    }else if ((0.00001 >=(1-c2)) && (-0.00001 <= (1-c2))){
        joint_positions[1] = atan2(0,c2);
        joint_positions[0] =  atan2(y,x) - atan2(link_length[1]*sin(joint_positions[1]),link_length[0] + link_length[1]*cos(joint_positions[1]));
        joint_positions[1] = -joint_positions[1];
    }
    else{
        ROS_ERROR("Target is out of range");
        return false;
    }

    if (z<0.05 && z>-0.04){
        joint_positions[2] = -z;
    }else{
        ROS_ERROR("Target is out of range");
        return false;
    }

    ROS_INFO("output joint positions %f %f %f",joint_positions[0],joint_positions[1],joint_positions[2]);
    return true;

}
void trajectoryExecutionCallback(const moveit_msgs::ExecuteTrajectoryActionResult result){

    ROS_INFO("....TRAJECTORY EXECUTION CALLBACK Status = %d....\n",result.status.status);
    if (result.status.status == 1 || result.status.status == 3){
        executionOK = true;
    }
}

int main(int argc, char **argv) {

    bool success;


    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nn;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber executeTrajectory = nn.subscribe("execute_trajectory/result", 1000, trajectoryExecutionCallback);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit::core::RobotStatePtr current_state;
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);


    //Ziskanie aktualne pozicie
    geometry_msgs::PoseStamped ws1 = move_group.getCurrentPose();
    ROS_INFO("End effector pose [ x=%f , y=%f , z=%f ]", ws1.pose.position.x, ws1.pose.position.y,
             ws1.pose.position.z);


    //Na overenie limitov klbov
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    boost::shared_ptr<urdf::ModelInterface> urdf;
    getchar();
    joint_limits_interface::JointLimits limits;



    getOffsets();



    int mode = 1;
    geometry_msgs::Pose points;
    std::vector<geometry_msgs::Pose> waypoints(2);
    moveit_msgs::RobotTrajectory trajectory_msg;

    double desired_x,desired_y,desired_z;

    while (ros::ok()){

        if (mode == 1) {
            ROS_INFO("Input X");
            scanf("%lf", &desired_x);
            ROS_INFO("Input Y");
            scanf("%lf", &desired_y);
            ROS_INFO("Input Z");
            scanf("%lf", &desired_z);
        }
        ROS_INFO("%f %f %f",desired_x,desired_y,desired_z);


        if (countIK(desired_x,desired_y,desired_z, mode)){
            ROS_INFO("IK start");

            move_group.setJointValueTarget(joint_positions);
            kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
            ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

            if (kinematic_state->satisfiesBounds()){
                mode = 1;

                success = move_group.plan(my_plan);
                //ROS_INFO("plan!!!");
                //ROS_INFO_STREAM(my_plan.trajectory_);
                //getchar();

                if (success){
                    ROS_INFO("Succesful plan!");
                    move_group.execute(my_plan);
                    move_group.move();
                    //Tu urobit odosielanie tych pozicii..

                    ROS_INFO("moved to place!!\n");
                } else{
                    ROS_ERROR("Bad plan");
                }
                ws1 = move_group.getCurrentPose();
                ROS_INFO("Effector x=%f , y=%f , z=%f \n", ws1.pose.position.x, ws1.pose.position.y, ws1.pose.position.z);
            }else{
                ROS_WARN("Colision warining! changing mode");
                mode++;
                if (mode >3){
                    break;
                }
            }
        }else{
            ROS_ERROR("No solution found");
        }


    }








    return 0;
}
