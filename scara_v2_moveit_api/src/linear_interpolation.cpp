//
// Created by viktor on 17/08/17.
//


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

using namespace std;

std::vector<double> joint_positions(3);
std::vector<double> link_length(2);
double x_offset, y_offset, z_offset;
geometry_msgs::Point point;

std::vector<geometry_msgs::Pose> setPointToWaypoints(std::vector<geometry_msgs::Pose> waypoints){

    geometry_msgs::Pose points;
    //Start position 1
    points.position.x = 0.59677;
    points.position.y = 0.78055;
    points.position.z = 1.0198;
    points.orientation.x = 0.29045;
    points.orientation.y = 0.95689;
    points.orientation.z = 0.00015729;
    points.orientation.w = -0.00065183;
    waypoints.push_back(points);

    //Finish position 1
    points.position.x = 0.31352;
    points.position.y = 0.78075;
    points.position.z = 1.0198;
    points.orientation.x =-0.32139;
    points.orientation.y = 0.94695;
    points.orientation.z = 0.00060969;
    points.orientation.w =-0.00089261;
    waypoints.push_back(points);

//    for (int i=0;i<waypoints.size();i++){
//        ROS_INFO_STREAM(waypoints[i]);
//    }
//    sleep(2);

    return waypoints;


}
bool positionControll (moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan, geometry_msgs::Pose position){

    bool success;
    int numberOfAttempts =0;


    while (numberOfAttempts<3) {
        if (move_group->setApproximateJointValueTarget(position, "tool0")) {
            ROS_INFO("found IK solution");
            break;
        } else
            ROS_INFO("only aproximate IK solution");
        numberOfAttempts++;
    }
    success = static_cast<bool>(move_group->plan(my_plan));
    if(success){

        int size=my_plan.trajectory_.joint_trajectory.points.size();

        ROS_INFO_STREAM("moje IK je");
        ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points[size-1]);
    }else{
        ROS_ERROR("could not create plan!");
        return 0;
    }
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 -back (pose goal) %s", success ? "GOOD" : "BAD");

    move_group->execute(my_plan);
    move_group->move();
    ROS_INFO("\n\nmy plan");
    ROS_INFO_STREAM(my_plan.trajectory_);
    return 1;


}
bool countIK(double x, double y, double z,  int mode){

    ROS_INFO("input numbers %f %f %f",x,y,z);
    x = x - x_offset;
    y = y - y_offset;
    z = z - z_offset;

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

int main(int argc, char **argv) {

    bool success;

    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit_msgs::RobotTrajectory trajectory_msg;


    geometry_msgs::Pose kkt;
    //0.70476; 0.58; 1.0196
    kkt.position.x = 0.70476;
    kkt.position.y = 0.58;
    kkt.position.z =  1.0196;
    //rpy+w
    //0.70711; 0.7071; -0.00027767; -0.00028542
    kkt.orientation.x = 0.70711;
    kkt.orientation.y = 0.7071;
    kkt.orientation.z = -0.00027767;
    kkt.orientation.w = -0.00028542;
//    ROS_INFO_STREAM(kkt);
//    getchar();


    //ROS_INFO("overenie klbov");
    //Na overenie limitov klbov
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    //getchar();

    current_state = move_group.getCurrentState();
    ROS_WARN("moving to default pose!!");
    getOffsets();
    countIK(kkt.position.x,kkt.position.y,kkt.position.z, 1);
    move_group.setJointValueTarget(joint_positions);
    kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
    success = static_cast<bool>(move_group.plan(my_plan));

    if (success){
        ROS_INFO("Succesful plan!");
        move_group.execute(my_plan);
        move_group.move();
        ROS_INFO("moved to place!!\n");
    } else{
        ROS_ERROR("Bad plan");
    }
    geometry_msgs::PoseStamped ws1 = move_group.getCurrentPose();
    ROS_INFO("Effector x=%f , y=%f , z=%f \n", ws1.pose.position.x, ws1.pose.position.y, ws1.pose.position.z);
    ROS_INFO("Press any key");
    getchar();




    std::vector<geometry_msgs::Pose> waypoints;
    waypoints = setPointToWaypoints(waypoints);

    //Solve IK from points (clasic solver)
    if (positionControll(&move_group, my_plan, waypoints[1])){
        ROS_INFO("Visualised movement to position 2");
    }else{
        ROS_INFO("Could not visualise movement to position 2");
    }
    ROS_INFO("Press any key");
    getchar();
    if (positionControll(&move_group, my_plan, waypoints[0])){
        ROS_INFO("Visualised movement to position 1");
        //ROS_INFO("Plan for the First position is ");
        //ROS_INFO_STREAM(my_plan.trajectory_);
    }else{
        ROS_INFO("Could not visualise movement to position 1");
    }
    ROS_INFO("Press any key");
    getchar();



    ROS_INFO("linear interpolation");
    //linear interpolation!
    double fraction = move_group.computeCartesianPath(waypoints,0.1,0.0,trajectory_msg, true);
    ROS_INFO("computed fraction = %f",fraction);
   // getchar();
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(),"scara_arm");
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory_msg);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    move_group.setJointValueTarget(joint_positions);
    kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);

    rt.getRobotTrajectoryMsg(trajectory_msg);
    ROS_INFO("\nWaypoints");
    for (int i =0;i<waypoints.size();i++){
        ROS_INFO_STREAM(waypoints[i]);
    }
    ROS_INFO("\ntrajectory_msg");
    ROS_INFO_STREAM(trajectory_msg.joint_trajectory);
    ROS_INFO("Press any key");
    getchar();

    my_plan.trajectory_ = trajectory_msg;
    if (fraction >= 0.5) {
        ROS_WARN("Plan %f percents",fraction);
        move_group.execute(my_plan);
        move_group.move();
        ROS_INFO("moving there!!!!");
    }
    else
        ROS_WARN("Could not compute the cartesian path :( <0.5 ");



    ros::shutdown();



        return 0;

    }