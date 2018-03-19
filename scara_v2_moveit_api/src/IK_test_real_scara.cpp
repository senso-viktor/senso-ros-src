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
#include "std_msgs/Byte.h"


using namespace std;

std::vector<double> joint_positions(3);
std::vector<double> actual_joint_positions(3);
std::vector<double> link_length(2);
double x_offset, y_offset, z_offset;
geometry_msgs::Point point;
bool executionOK = true;
geometry_msgs::Pose pos_and_vel;
geometry_msgs::Point acc;

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
    //ROS_INFO("final offsets: x_offset=%f y_offset=%f z_offset=%f",x_offset, y_offset, z_offset);
    //ROS_INFO("final arm lenths: arm1=%f arm2=%f",link_length[0], link_length[1]);
    sleep(2);

}

bool countIK(double x, double y, double z,  int mode){

    //ROS_INFO("input numbers %f %f %f",x,y,z);
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
void jointStatesCallback (const sensor_msgs::JointState jointStates){

    //ROS_INFO("Joint states callback");
    actual_joint_positions[0] = jointStates.position[1];
    actual_joint_positions[1] = jointStates.position[2];
    actual_joint_positions[2] = jointStates.position[3];
    //ROS_INFO("%f %f %f",actual_joint_positions[1],actual_joint_positions[2],actual_joint_positions[3]);

}

void sendJointPoses(ros::Publisher *pose_and_vel_pub,ros::Publisher *accel_pub, moveit::planning_interface::MoveGroupInterface::Plan *plan, int i){

    if (i == 999){
        pos_and_vel.position.x = 0.0;
        pos_and_vel.position.y =  0.0;
        pos_and_vel.position.z =  0.0;
        pos_and_vel.orientation.x =  0.0;
        pos_and_vel.orientation.y =  0.0;
        pos_and_vel.orientation.z =  0.0;
        acc.x = 0.0;
        acc.y = 0.0;
        acc.z = 0.0;
    }else{
        pos_and_vel.position.x = plan->trajectory_.joint_trajectory.points[i].positions[0];
        pos_and_vel.position.y = plan->trajectory_.joint_trajectory.points[i].positions[1];
        pos_and_vel.position.z = plan->trajectory_.joint_trajectory.points[i].positions[2];
        pos_and_vel.orientation.x = plan->trajectory_.joint_trajectory.points[i].velocities[0];
        pos_and_vel.orientation.y = plan->trajectory_.joint_trajectory.points[i].velocities[1];
        pos_and_vel.orientation.z = plan->trajectory_.joint_trajectory.points[i].velocities[2];
        acc.x = plan->trajectory_.joint_trajectory.points[i].accelerations[0];
        acc.y = plan->trajectory_.joint_trajectory.points[i].accelerations[1];
        acc.z = plan->trajectory_.joint_trajectory.points[i].accelerations[2];
    }

    pose_and_vel_pub->publish(pos_and_vel);
    accel_pub->publish(acc);

}

int main(int argc, char **argv) {

    bool success;


    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle n1,nn1,n2,nn;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(15);

    double maxJointDeviation = 0.1;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);


    ros::Publisher pose_pub = n1.advertise<geometry_msgs::Pose>("/planned_poses_and_velocities",1000);
    ros::Publisher acc_pub = nn1.advertise<geometry_msgs::Point>("/planned_accelerations",1000);
    ros::Publisher mode_pub = n2.advertise<std_msgs::Byte>("/modeSelect",1000);
    ros::Subscriber subscribe_realJointValues = nn.subscribe("joint_states",1000,jointStatesCallback);

    //Ziskanie aktualne pozicie
    geometry_msgs::PoseStamped ws1 = move_group.getCurrentPose();
    //ROS_INFO("End effector pose [ x=%f , y=%f , z=%f ]", ws1.pose.position.x, ws1.pose.position.y,
     //        ws1.pose.position.z);

    //Na overenie limitov klbov
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    getOffsets();
    int mode = 1;
    geometry_msgs::Pose points;
    std::vector<geometry_msgs::Pose> waypoints(2);
    moveit_msgs::RobotTrajectory trajectory_msg;
    std_msgs::Byte selectedMode;
    selectedMode.data = 6;

    double desired_x,desired_y,desired_z;
    int last_trajectory_size = -5;
    int i = 0 ;

    for (int j = 0; j < 50; j++){
        sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
        mode_pub.publish(selectedMode);
        ROS_INFO("Init matlab");
        usleep(50000);
    }

    while (ros::ok()){

        mode_pub.publish(selectedMode);

        if ((joint_positions[0]-maxJointDeviation < actual_joint_positions[0]) && (actual_joint_positions[0] < joint_positions[0]+maxJointDeviation)){
            if ((joint_positions[1]-maxJointDeviation < actual_joint_positions[1]) && (actual_joint_positions[1] < joint_positions[1]+maxJointDeviation)){
                if ((joint_positions[2]-maxJointDeviation < actual_joint_positions[2]) && (actual_joint_positions[2] < joint_positions[2]+maxJointDeviation)){
                    ROS_WARN("!!!!!   In place  !!!!!!");
                    for (int i=0;i<joint_positions.size();i++){
                        ROS_ERROR("Desired joint %d value %f",i,joint_positions[i]);
                        ROS_ERROR("Joint %d value %f",i,actual_joint_positions[i]);
                        executionOK = true;
                    }

                } else
                    ROS_INFO("J3 not in place %f [%f]",actual_joint_positions[2],joint_positions[2]);
            }else
                ROS_INFO("J2 not in place %f [%f]",actual_joint_positions[1],joint_positions[1]);
        }else
            ROS_INFO("J1 not in place %f [%f]",actual_joint_positions[0],joint_positions[0]);


        if (executionOK){
            executionOK = false;
            //Enter desired pose
            if (mode == 1) {
                ROS_INFO("Input X");
                scanf("%lf", &desired_x);
                if (desired_x == 0.0)
                    break;
                ROS_INFO("Input Y");
                scanf("%lf", &desired_y);
                if (desired_y == 0.0)
                    break;
                ROS_INFO("Input Z");
                scanf("%lf", &desired_z);
                if (desired_z == 0.0)
                    break;
            }
            ROS_INFO("%f %f %f",desired_x,desired_y,desired_z);

            while (1){
                if (countIK(desired_x,desired_y,desired_z, mode)){
                    move_group.setJointValueTarget(joint_positions);
                    kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
                    if (kinematic_state->satisfiesBounds()){
                        mode = 1;
                        success = static_cast<bool>(move_group.plan(my_plan));
                        if (success){
                            ROS_INFO("Succesful plan!.. now moving to place");
                            //Asynchrone vykonavanie
                            move_group.asyncExecute(my_plan);
                            move_group.asyncMove();
                            break;
                        } else{
                            ROS_ERROR("Bad plan");
                            executionOK = true;
                            break;
                        }
                        ws1 = move_group.getCurrentPose();
                        //ROS_INFO("Effector x=%f , y=%f , z=%f \n", ws1.pose.position.x, ws1.pose.position.y, ws1.pose.position.z);
                    }else{
                        ROS_WARN("Colision warining! changing mode");
                        mode++;
                        if (mode >3){
                            executionOK = true;
                            ROS_INFO("Cannot solve IK please enter new positions");
                            break;
                        }
                    }
                }else{
                    ROS_ERROR("No solution found");
                    ROS_INFO("Cannot solve IK please enter new positions");
                    executionOK = true;
                    break;
                }

            }

        }

        //ROS_INFO("Size of planned trajectory %d",);

        if (my_plan.trajectory_.joint_trajectory.points.size() != last_trajectory_size){
            last_trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
            i=0;
        }

        if (!executionOK){
            if (i< my_plan.trajectory_.joint_trajectory.points.size())
            {
                sendJointPoses(&pose_pub,&acc_pub, &my_plan, i);
                ROS_WARN("message GO! %f %f %f [%d/%d]",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z,i,last_trajectory_size);
                i++;
            }else{
                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                ROS_ERROR("message stay!![%f %f %f]",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
            }

        }

        ros::spinOnce();
        loop_rate.sleep();

    }








    return 0;
}
