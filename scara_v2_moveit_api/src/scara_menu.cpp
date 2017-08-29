//
// Created by viktordluhos on 28/08/17.
//

#include "../include/scara_menu.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "menu_node");
    ros::NodeHandle n1,n2,n3,n4,n5,n6;
    ros::NodeHandle nn1,nn2,nn3,nn4,nn5,nn6;
    ros::Rate loop_rate(5);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //interaction with moveit
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);

    //Na overenie limitov klbov
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    //Pre position control
    getOffsets();



    //Publishers
    ros::Publisher actualPose_pub = n1.advertise<geometry_msgs::Pose>("actualPose",1000);
//    ros::Publisher infoPublisher = n1.advertise<scara_v2_moveit_api::scara_basic_info>("scara/basic_info", 1000);
    //Subscriber
    ros::Subscriber modeSelect_sub = nn1.subscribe("modeSelect",1000,modeSelectCallback);
    ros::Subscriber startState_sub = nn2.subscribe("startState",1000, startStateCallback);
    ros::Subscriber gripperState_sub = nn3.subscribe("gripperState",1000, gripperStateCallback);
    ros::Subscriber jointControl_sub = nn4.subscribe("jointControl",1000, jointControlCallback);
    ros::Subscriber positionControl_sub = nn5.subscribe("positionControl",1000,positionControlCallback);


    while (ros::ok()){
        ROS_INFO_ONCE("while start");

    switch (current_mode){
        case 0:
            while (ros::ok()){
                //Break while loop when the mode has changed
                if (current_mode != 0){
                    start_state = false;
                    move_group.stop();
                    break;
                }

                ROS_INFO("Information tab - do nothing");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;

        case 1:
            while (ros::ok()) {
                //Break while loop when the mode has changed
                if (current_mode != 1){
                    start_state = false;
                    move_group.stop();
                    break;
                }
                sendEndEffectorPose(&actualPose_pub, &move_group);
//                for (int i = 0;i<move_group.getCurrentJointValues().size();i++){
//                    std::vector<double> jointVal = move_group.getCurrentJointValues();
//                    ROS_INFO("Joint%d = %f",i,jointVal[i]);
//                }

                if (start_state) {
                    ROS_INFO("Joint control tab started");
                    sleep(0.5);
//                    ROS_INFO("Desired joints : %f %f %f",jointControl_jointValues[0],jointControl_jointValues[1],jointControl_jointValues[2]);
//                    ROS_INFO("gripper state %d",gripper_state);
                    if (valuesChanged()) {
                        ROS_WARN("Desired joints : %f %f %f", jointControl_jointValues[0], jointControl_jointValues[1],
                                 jointControl_jointValues[2]);
                        move_group.setJointValueTarget(jointControl_jointValues);
                        kinematic_state->setJointGroupPositions(joint_model_group, jointControl_jointValues);
                        if (!kinematic_state->satisfiesBounds()) {
                            ROS_ERROR("Bad input joint values");
                            start_state = false;
                            break;
                        }
                        jointModeControll(&move_group, my_plan);
                    }
                }else{
                    ROS_ERROR("Movement stopped!!");
                    //Otestovat
                    jointControl_lastJointValues[0] = 9.99;
                    //
                    move_group.stop();
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 2:
            while (ros::ok()){
                //Break while loop when the mode has changed
                if (current_mode != 2){
                    start_state = false;
                    move_group.stop();
                    break;
                }
                sendEndEffectorPose(&actualPose_pub, &move_group);
//                for (int i = 0;i<move_group.getCurrentJointValues().size();i++){
//                    std::vector<double> jointVal = move_group.getCurrentJointValues();
//                    ROS_INFO("Joint%d = %f",i,jointVal[i]);
//                }

                if (start_state) {
                    ROS_INFO("Position control start request");
                    if (positionsChanged()){
                        while (ros::ok()){
                            if (countIK(positionControl_values[0], positionControl_values[1], positionControl_values[2], IK_mode)){
                                move_group.setJointValueTarget(joint_positions);
                                kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
                                if (kinematic_state->satisfiesBounds()){
                                    IK_mode = 1;
                                    success = move_group.plan(my_plan);
                                    if (success){
                                        ROS_INFO("Succesful plan!.. now moving to place");
                                        move_group.asyncExecute(my_plan);
                                        break;
                                    } else{
                                        ROS_ERROR("Bad plan");
                                        start_state = false;
                                        break;
                                    }
                                }else{
                                    ROS_WARN("Colision warining! changing mode");
                                    IK_mode++;
                                    if (IK_mode >3){
                                        ROS_INFO("Cannot solve IK please enter new positions");
                                        start_state = false;
                                        break;
                                    }
                                }
                            }else{
                                ROS_ERROR("No solution found");
                                ROS_INFO("Cannot solve IK please enter new positions");
                                start_state = false;
                                break;
                            }
                        }
                     }
                }else {
                    ROS_ERROR("Movement stopped!!");
                    move_group.stop();
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;

        case 3:
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 3)
                    break;

                ROS_INFO("DEMO control tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 4:
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 4)
                    break;

                ROS_INFO("Information tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 5:
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 5)
                    break;

                ROS_INFO("Set parameters tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 6:         //Ked budem dorabat veci do GUI
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 7)
                    break;

                ROS_INFO("testing tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        default:
            //ROS_INFO("No mode selected!");
            break;
    }

        ros::spinOnce();
        loop_rate.sleep();
    }





    return 0;
}