//
// Created by viktordluhos on 28/08/17.
//

#include "../include/scara_menu.h"


void forceFeedbackThread(){

    ROS_INFO_ONCE("forcefeedback thread start");

    while (ros::ok()){

        if ((currentJointStates.effort[0] >= max_torque_value) || (currentJointStates.effort[1] >= max_torque_value)){
            colisionDetection = true;
            start_state = false;
            teach_start_state = false;
            //ROS_ERROR("STOP ! [J1_torq=%f J2_torq=%f start_state=%d]",currentJointStates.effort[0],currentJointStates.effort[1],start_state);
        }else{
            //ROS_INFO("colision detection = %d",colisionDetection);
            colisionDetection = false;
        }

        //ROS_INFO("[t]J1_torq=%f J2_torq=%f (max=%f)",currentJointStates.effort[0],currentJointStates.effort[1], max_torque_value);
        usleep(100000);
        ros::spinOnce();

    }
}

int main(int argc, char **argv){

    bool initStep = true;
    bool satisfieJointLimits = false;
    bool errorInPose = false;
    bool init = true;
    int counter1 = 0;
    bool initMode = false;
    int help = 0;
    int cc = -1;
    bool initPoses = true;
    int numOfPlacePos = 0;
    int desiredJointsTeachSize = 0;
    int teachPositionsHandSize = 0;

    ros::init(argc, argv, "menu_node");
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9,n10;
    ros::NodeHandle nn1,nn2,nn3,nn4,nn5,nn6,nn7,nn8,nn9,nn10,nn11,nn12,nn13,nn14,nn15,nn16,nn17,nn18;
    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();


    executionOK = true;


    //interaction with moveit
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    mg = &move_group;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);

    //Na overenie limitov klbov
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    //init arm length and offsets for position control
    getOffsets();


    ROS_INFO("Init publishers");
    //Publishers
    ros::Publisher actualPose_pub = n1.advertise<geometry_msgs::Pose>("actualPose",1000);
    ros::Publisher errorMessage_pub = n2.advertise<std_msgs::Int32>("errorCode",1000);
    ros::Publisher pose_pub = n3.advertise<geometry_msgs::Pose>("/planned_poses_and_velocities",1000);
    ros::Publisher acc_pub = n4.advertise<geometry_msgs::Point>("/planned_accelerations",1000);
    ros::Publisher mode_pub = n5.advertise<std_msgs::Byte>("/modeSelect",1000);
    ros::Publisher gripper_pub = n6.advertise<std_msgs::Byte>("/gripperCommand",1000);
    ros::Publisher centralStop_pub = n7.advertise<std_msgs::Int32>("centralStop",1000);
    ros::Publisher startMoveitMode_pub = n8.advertise<std_msgs::Bool>("moveitModeStart",1000);
    ros::Publisher sendInfo_pub = n9.advertise<scara_msgs::robot_info>("getInfoValues",1000);
    info_pub = &sendInfo_pub;
    ros::Publisher desiredPos_pub = n10.advertise<geometry_msgs::Point>("desiredPose",1000);
    desPos_pub = &desiredPos_pub;
    ROS_INFO("Init subscribers");
    //Subscriber
    ros::Subscriber modeSelect_sub = nn1.subscribe("modeSelectGUI",1000,modeSelectCallback);
    ros::Subscriber startState_sub = nn2.subscribe("startState",1000, startStateCallback);
    ros::Subscriber gripperState_sub = nn3.subscribe("gripperState",1000, gripperStateCallback);
    ros::Subscriber jointControl_sub = nn4.subscribe("jointControl",1000, jointControlCallback);
    ros::Subscriber positionControl_sub = nn5.subscribe("positionControl",1000,positionControlCallback);
    ros::Subscriber scaraJointStates_sub = nn6.subscribe("scara_jointStates",1000,jointStatesCallback);
    ros::Subscriber teachMode_sub = nn7.subscribe("teachModeGUI",1000,teachModeCallback);
    ros::Subscriber teachModeStartState_sub = nn8.subscribe("teachModeStartState",1000,teachModeStartStateCallback);
    ros::Subscriber buttonState_sub = nn9.subscribe("scara_pushbutton",1000,buttonStateCallback);
    ros::Subscriber centralStop_sub = nn10.subscribe("centralStop",1000,centralStopCallback);
    ros::Subscriber setTorque_sub = nn11.subscribe("setTorque",1000,setTorqueCallback);
    ros::Subscriber setVel_sub = nn12.subscribe("setVelocity",1000,setVelCallback);
    ros::Subscriber setAcc_sub = nn13.subscribe("setAcceleration",1000,setAccCallback);
    ros::Subscriber setPlanTime_sub = nn14.subscribe("setPlanningTime",1000,setPlanTimeCallback);
    ros::Subscriber setNumOfAttempts_sub = nn15.subscribe("setNumberOfAttempts",1000,setNumOfAttemptsCallback);
    ros::Subscriber moveitMode_sub = nn16.subscribe("moveitModeStart",1000,moveitModeCallback);
    ros::Subscriber setPrecision_sub = nn17.subscribe("setPrecision", 1000, setPrecisionCallback);
    ros::Subscriber info_sub = nn18.subscribe("getInfo",1000,infoCallback);
    sleep(2);

    //init desiredPositions message (for GUI)
    sendPositionToGUI(0,0,0);

    //Set desired Poses for DEMO
    setDesiredPosesDEMO();

    //Calculate desired angles for DEMO
    for (int i=0; i<desiredJointsDEMO.size(); i++){

        while (ros::ok()){
            if (calculateIK(desiredPositionsDEMO[i].x, desiredPositionsDEMO[i].y, desiredPositionsDEMO[i].z, IK_mode, 1 , i)){
                move_group.setJointValueTarget(joint_positions);
                kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
                if (kinematic_state->satisfiesBounds()){
                    IK_mode = 1;
                    break;
                }else{
                    //ROS_WARN("Colision warining! changing mode");
                    IK_mode++;
                    if (IK_mode >3){
                        //ROS_INFO("Cannot solve IK please enter new positions");
                        desiredJointsDEMO[i][0] = 0.0;
                        desiredJointsDEMO[i][1] = 0.0;
                        desiredJointsDEMO[i][2] = 0.0;
                        ROS_ERROR("error in %d",i);
                        errorInPose = true;

                        break;
                    }
                }
            }else{
                //ROS_INFO("Cannot solve IK please enter new positions");
                desiredJointsDEMO[i][0] = 0.0;
                desiredJointsDEMO[i][1] = 0.0;
                desiredJointsDEMO[i][2] = 0.0;
                ROS_ERROR("error in %d",i);
                errorInPose = true;
                break;
            }
        }


    }
    if (errorInPose){
        ROS_INFO("Error in computing IK for pose (DEMO)");
    }else{
        ROS_INFO("computing IK OK (DEMO)");
    }
//    ROS_INFO("Final angles:");
//    for (int i=0; i<desiredJointsDEMO.size(); i++){
//        ROS_INFO("[%d] J1=%f J2=%f J3=%f",i,desiredJointsDEMO[i][0],desiredJointsDEMO[i][1],desiredJointsDEMO[i][2]);
//    }
    //move_group.setPlannerId("RRTConnectkConfigDefault");
    sleep(2);


    ROS_INFO("wait for subscribers");
    //Wait for publishers on specified topics
    waitForPublishers(&scaraJointStates_sub, 1);
    waitForPublishers(&modeSelect_sub, 1);
    waitForPublishers(&startState_sub, 1);
    waitForPublishers(&gripperState_sub, 1);
    waitForPublishers(&jointControl_sub, 1);
    waitForPublishers(&positionControl_sub, 1);
    waitForPublishers(&teachMode_sub, 1);
    waitForPublishers(&teachModeStartState_sub, 1);
    sleep(2);
    ROS_INFO("wait for SCARA and Matlab init");
    for (int i=0;i<20;i++){
        initMatlab(&gripper_pub, &centralStop_pub, &pose_pub, &acc_pub, &my_plan);
        ros::spinOnce();
        loop_rate.sleep();
    }
    sleep(6);



    //Paralel thread for Colision detection START
    boost::thread fft{forceFeedbackThread};

    while (ros::ok()){
        ROS_INFO_ONCE("while start");

        if (initPoses){
            for (int i=0;i<10;i++){
                sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
            }
            initPoses = false;
        }

        if (central_stop){
            selectedMode.data = 6;
            mode_pub.publish(selectedMode);
            for (int i=0;i<10;i++)
                sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
            ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
            return 0;
        }



        switch (current_mode){

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////                                     MODE 0 - info                                                   /////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 0:
                while (ros::ok()){
                    //Break while loop when the mode has changed
                    if (current_mode != 0){
                        start_state = false;
                        move_group.stop();
                        gripper_state.data = 0;
                        gripper_pub.publish(gripper_state);
                        break;
                    }

                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        desiredPositions.x = 0.0;
                        desiredPositions.y = 0.0;
                        desiredPositions.z = 0.0;
                        //desPos_pub->publish(desiredPositions);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }

                    //ROS_INFO("Information tab - do nothing");
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                break;

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 1 - Joint Control                                          /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 1:
                while (ros::ok()) {
                    //Break while loop when the mode has changed
                    if (current_mode != 1){
                        start_state = false;
                        move_group.stop();
                        count1 = 0;
                        jointControl_counter = 0;
                        last_trajectory_size = -5;
                        satisfieJointLimits = false;
                        gripper_state.data = 0;
                        gripper_pub.publish(gripper_state);
                        break;
                    }
                    //Publish current pose of end effector (for GUI)
                    selectedMode.data = 6;
                    mode_pub.publish(selectedMode);
                    gripper_pub.publish(gripper_state);
                    if (count1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        count1 = 0;
                    }
                    count1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }


                    if (start_state && !colisionDetection) {
                        //ROS_INFO_ONCE("Joint control tab started");

                        if (valuesChanged()) {
                            ROS_WARN("Desired joints : %f %f %f", jointControl_jointValues[0], jointControl_jointValues[1],
                                     jointControl_jointValues[2]);
                            jointControl_counter = 0;
                            move_group.setJointValueTarget(jointControl_jointValues);
                            kinematic_state->setJointGroupPositions(joint_model_group, jointControl_jointValues);
                            satisfieJointLimits = kinematic_state->satisfiesBounds();
                            if (!kinematic_state->satisfiesBounds()) {
                                //ROS_ERROR("Bad input joint values");
                                sendErrorCode(&errorMessage_pub, 1);
                                start_state = false;
                                break;
                            }
                            if (jointModeControll(&move_group)){
                                ROS_INFO("OKAY %d",success);
                            }else{
                                ROS_INFO("Somethng wrong with the function %d",success);
                                break;  //pridal som break
                            }
                        }

                        if (satisfieJointLimits){
                            ROS_INFO("Able to move");
                            if (my_plan.trajectory_.joint_trajectory.points.size() != last_trajectory_size){
                                last_trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
                                ROS_ERROR("SIZE OF PLAN : %d",last_trajectory_size);
                                jointControl_counter=0;
                            }
                            if (jointControl_counter < my_plan.trajectory_.joint_trajectory.points.size()){
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, jointControl_counter);
                                ROS_WARN("message GO! [%d/%d]",jointControl_counter,last_trajectory_size);
                                jointControl_counter++;
                            }else{
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                ROS_ERROR("message stay!!",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
                            }
                        }else{
                            ROS_INFO("Not able to move bouds=%d succes=%d",satisfieJointLimits, success);
                        }

                    }else{
                        //ROS_ERROR("Movement stopped!!");
                        //ROS_INFO("size of plan %d", my_plan.trajectory_.joint_trajectory.points.size());
                        if (jointControl_counter < 1){
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                            ROS_ERROR("message stop!! [0/%d]",last_trajectory_size);
                        }else if (jointControl_counter < 15){
                            if (jointControl_counter == last_trajectory_size){
                                jointControl_counter = last_trajectory_size - 1;
                            }
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, jointControl_counter);
                            ROS_ERROR("message stop!! [%d/%d]", jointControl_counter,last_trajectory_size);
                        }else{
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, (jointControl_counter-10));
                            ROS_ERROR("message stop!! [%d/%d]", jointControl_counter-10,last_trajectory_size);
                        }

                        //ROS_ERROR("message stop!! [%d/%d]", jointControl_counter-10,last_trajectory_size);
                        jointControl_lastJointValues[0] = 9.99;
                        move_group.stop();
                    }
                    //ROS_INFO("start_state = %d  colisionDetection = %d",start_state,colisionDetection);

                    ros::spinOnce();
                    loop_rate.sleep();
                }

                break;

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 2 - Position control                                       /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 2:

                while (ros::ok()){
                    //Break while loop when the mode has changed
                    if (current_mode != 2){
                        start_state = false;
                        move_group.stop();
                        count1 = 0;
                        positionControl_counter = 0;
                        last_trajectory_size = -5;
                        IK_mode = 1;
                        satisfieJointLimits = false;
                        gripper_state.data = 0;
                        gripper_pub.publish(gripper_state);
                        break;
                    }
                    //Publish current pose of end effector (for GUI)
                    selectedMode.data = 6;
                    mode_pub.publish(selectedMode);
                    gripper_pub.publish(gripper_state);
                    if (count1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        count1 = 0;
                    }
                    count1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }

                    if (start_state && !colisionDetection) {
                        if (positionsChanged()){
                            while (ros::ok()){
                                if (calculateIK(positionControl_values[0], positionControl_values[1], positionControl_values[2], IK_mode, 0, 0)){
                                    move_group.setJointValueTarget(joint_positions);
                                    kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
                                    satisfieJointLimits = kinematic_state->satisfiesBounds();
                                    if (kinematic_state->satisfiesBounds()){
                                        IK_mode = 1;
                                        success = move_group.plan(my_plan);
                                        if (success){
                                            ROS_INFO("Succesful plan! .. moving to place");
                                            move_group.asyncExecute(my_plan);
                                            if (my_plan.trajectory_.joint_trajectory.points.size() != last_trajectory_size){
                                                last_trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
                                                positionControl_counter=0;
                                            }
                                            desiredPositions.x = positionControl_values[0];
                                            desiredPositions.y = positionControl_values[1];
                                            desiredPositions.z = positionControl_values[2];
                                            //desPos_pub->publish(desiredPos_pub);

                                            break;
                                        } else{
                                            ROS_ERROR("Bad plan");
                                            sendErrorCode(&errorMessage_pub, 2);
                                            start_state = false;
                                            break;
                                        }
                                    }else{
                                        ROS_WARN("Colision warining! changing mode");
                                        sendErrorCode(&errorMessage_pub, 3);
                                        IK_mode++;
                                        if (IK_mode >3){
                                            ROS_INFO("Cannot solve IK please enter new positions");
                                            sendErrorCode(&errorMessage_pub, 4);
                                            start_state = false;
                                            satisfieJointLimits = false;
                                            break;
                                        }
                                    }
                                }else{
                                    ROS_ERROR("No solution found");
                                    ROS_INFO("Cannot solve IK please enter new positions");
                                    sendErrorCode(&errorMessage_pub, 5);
                                    start_state = false;
                                    satisfieJointLimits = false;
                                    break;
                                }
                            }

                        }

//
                        if (success && satisfieJointLimits){
                            ROS_INFO("Able to move!!!!!");
                            if (positionControl_counter < my_plan.trajectory_.joint_trajectory.points.size()){
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, positionControl_counter);
                                ROS_WARN("message GO! [%d/%d]",positionControl_counter,last_trajectory_size);
                                positionControl_counter++;
                            }else{
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                ROS_ERROR("message stay!!",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
                            }
                        }else{
                            ROS_INFO("Not able to move due to bad plan or so..");
                        }

                    }else{
                        //ROS_ERROR("Movement stopped!!");

                        if (positionControl_counter < 1){
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                            ROS_ERROR("message stop!! [0/%d]",last_trajectory_size);
                        }else if (positionControl_counter < 15){
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, positionControl_counter);
                            ROS_ERROR("message stop!! [%d/%d]", positionControl_counter,last_trajectory_size);
                        }else{
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, (positionControl_counter-10));
                            ROS_ERROR("message stop!! [%d/%d]", (positionControl_counter-10),last_trajectory_size);
                        }
                        positionControl_lastValues[0] = 9.99;
                        move_group.stop();
                    }
                    //ROS_INFO("start_state = %d colisionDetection = %d",start_state,colisionDetection);


                    ros::spinOnce();
                    loop_rate.sleep();
                }
                break;

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 3 - DEMO                                                   /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 3:
                while (ros::ok()){
                    ROS_INFO_ONCE("DEMO control tab ...");
                    //Break while loop when the mode has changed
                    if (current_mode != 3){
                        move_group.stop();
                        count1 = 0;
                        demoControl_counter = 0;
                        last_trajectory_size = -5;
                        start_state = false;
                        executionOK = true;
                        DEMO_mode = -1; //or 0....
                        gripper_state.data = 0;
                        gripper_pub.publish(gripper_state);
                        init = true;
                        pick = true;
                        break;
                    }

                    selectedMode.data = 6;
                    mode_pub.publish(selectedMode);
                    gripper_pub.publish(gripper_state);
                    //Publish current pose of end effector (for GUI)
                    if (count1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        count1 = 0;
                    }
                    count1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }

                    if (start_state && !colisionDetection){       //colision!!!!!

                        //ROS_INFO("running");
                        //executionOK = inPosition(DEMO_mode);

                        if (executionOK){

                            sleep(2);
                            DEMO_mode++;

                            if (DEMO_mode == 2){            //pick
                                pick = true;
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                usleep(1000000);
                                gripper_state.data = 1;
                                gripper_pub.publish(gripper_state);
                                usleep(400000);
                                pick = false;
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                            }
                            if (DEMO_mode >= 4){            //place
                                pick = true;
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                usleep(1000000);
                                gripper_state.data = 0;
                                gripper_pub.publish(gripper_state);
                                usleep(400000);
                                pick = false;
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                            }




                            if (DEMO_mode > 3){
                                DEMO_mode = 0;  //home position
                            }
                            if (DEMO_mode == 3){    //for placing positions
                                if (numOfPlacePos == 7)
                                    numOfPlacePos = 0;
                                DEMO_mode += numOfPlacePos;
                                numOfPlacePos++;

                            }

                            ROS_WARN("Moving to[%d]: %f %f %f",DEMO_mode, desiredJointsDEMO[DEMO_mode][0], desiredJointsDEMO[DEMO_mode][1],
                                     desiredJointsDEMO[DEMO_mode][2]);
                            sleep(1);
                            demoControl_counter = 0;
                            satisfieJointLimits = move_group.setJointValueTarget(desiredJointsDEMO[DEMO_mode]);
                            jointModeControll(&move_group);
                        }

                        executionOK = inPosition(DEMO_mode);

                        if (satisfieJointLimits && success){
                            ROS_INFO("Able to move");
                            if (my_plan.trajectory_.joint_trajectory.points.size() != last_trajectory_size){
                                last_trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
                                demoControl_counter=0;
                            }

                            if (demoControl_counter < my_plan.trajectory_.joint_trajectory.points.size()){
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, demoControl_counter);
                                ROS_WARN("message GO! [%d/%d]",demoControl_counter,last_trajectory_size);
                                demoControl_counter++;
                            }else{
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                ROS_ERROR("message stay!!",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
                            }
                        }else{
                            ROS_INFO("Not able to move bouds:%d success:%d",satisfieJointLimits,success);
                            executionOK = true;
                            ROS_ERROR("Due to bad position moving to next position!");
                        }


                    }else{
                        //ROS_INFO("stopped");
                        if (demoControl_counter < 1){
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                            ROS_ERROR("message stop!! [0/%d]",last_trajectory_size);
                        }else if (demoControl_counter < 15){
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, jointControl_counter);
                            ROS_ERROR("message stop!! [%d/%d]", demoControl_counter-10,last_trajectory_size);
                        }else{
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, (demoControl_counter-10));
                            ROS_ERROR("message stop!! [%d/%d]", (demoControl_counter-10),last_trajectory_size);
                        }
                        positionControl_lastValues[0] = 9.99;
                        move_group.stop();
                    }

                    ros::spinOnce();
                    loop_rate.sleep();
                }
                break;


                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 4 - teach mode GUI                                        /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 4:
                while (ros::ok()){
                    ROS_INFO_ONCE("teaching mode");

                    //Break while loop when the mode has changed
                    if (current_mode != 4){
                        move_group.stop();
                        count1 = 0;
                        last_trajectory_size = -5;
                        start_state = false;
                        teach_start_state = false;
                        initTeachedPositions = true;
                        IK_mode = 1;
                        zeroPositionForTeach = true;
                        teachMode_counter = 0;
                        initTeachedPositions = true;
                        executionOK = true;
                        teachPositionsHand.clear();
                        teachPositions.clear();
                        teach_mode = 9;
                        help = 0;
                        counter1 = 0;
                        desiredJointsTeachSize = 0;
                        break;
                    }

                    selectedMode.data = 6;
                    mode_pub.publish(selectedMode);
                    gripper_pub.publish(gripper_state);

                    if (counter1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        counter1 = 0;
                    }
                    counter1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }

                    if (teach_mode == 0){           //teach mode
                        //desiredJointsTeach.clear();
                        ROS_INFO_ONCE("teaching now");
                        if (start_state){     //if teach button was pushed
                            if (teachPointChanged()){
                                ROS_WARN("teached new positions!!!!");
                                teachPositions.push_back(currentTeachPoint);
                                start_state = false;
                                help =1;
                            }
                            count1 = 0;

                        }


                    }else if (teach_mode == 1)   {                      //run mode

                        //Set mode for joint control
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);

                        if (initTeachedPositions && (help == 1)){          //init vector and  calculate IK
                            ROS_WARN("\n\nstopped teaching %d",desiredJointsTeach.size());
                            showAndInitVector();
                            for (int i=0; i<desiredJointsTeach.size(); i++){

                                while (ros::ok()){
                                    if (calculateIK(teachPositions[i].x, teachPositions[i].y, teachPositions[i].z, IK_mode, 2 , i)){
                                        move_group.setJointValueTarget(joint_positions);
                                        kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
                                        if (kinematic_state->satisfiesBounds()){
                                            IK_mode = 1;
                                            break;
                                        }else{
                                            ROS_WARN("Colision warining! changing mode");
                                            IK_mode++;
                                            if (IK_mode >3){
                                                ROS_INFO("Cannot solve IK please enter new positions");
                                                desiredJointsTeach[i][0] = 0.0;
                                                desiredJointsTeach[i][1] = 0.0;
                                                desiredJointsTeach[i][2] = 0.0;
                                                break;
                                            }
                                        }
                                    }else{
                                        ROS_INFO("Cannot solve IK please enter new positions");
                                        desiredJointsTeach[i][0] = 0.0;
                                        desiredJointsTeach[i][1] = 0.0;
                                        desiredJointsTeach[i][2] = 0.0;
                                        break;
                                    }
                                }
                            }
                            ROS_WARN("\n\nstopped teaching %d",desiredJointsTeach.size());
                            desiredJointsTeachSize = desiredJointsTeach.size();
                            if (!(desiredJointsTeachSize%2 == 0)){
                                desiredJointsTeachSize--;
                                ROS_INFO("size not ok");
                            }else{
                                ROS_INFO("size ok");
                            }

                            for (int i=0; i<desiredJointsTeachSize; i++){
                                ROS_WARN("[%d] J1=%f J2=%f J3=%f",i,desiredJointsTeach[i][0],desiredJointsTeach[i][1],desiredJointsTeach[i][2]);
                            }
                            initTeachedPositions = false;
                            sleep(2);
                        }


                        if (teach_start_state && !colisionDetection){



                            ROS_INFO("cout = %d  executionOK = %d",count1,executionOK);


                            if (executionOK){

                                sleep(2);
                                ROS_WARN("Desired target[%d] : %f %f %f",count1, desiredJointsTeach[count1][0], desiredJointsTeach[count1][1],
                                         desiredJointsTeach[count1][2]);
                                move_group.setJointValueTarget(desiredJointsTeach[count1]);
                                jointModeControll(&move_group);
                            }



                            executionOK = inPositionTeach(count1,1);
                            ROS_INFO("count = %d",count1);


                            if (executionOK){       //HERE ENDED

                                if (count1 == 0){
                                    ROS_WARN("PICK");
                                    pick = true;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                    usleep(1000000);
                                    gripper_state.data = 1;
                                    gripper_pub.publish(gripper_state);
                                    usleep(400000);
                                    pick = false;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                }
                                if (count1%2 == 0){         //Pick
                                    ROS_WARN("PICK");
                                    pick = true;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                    usleep(1000000);
                                    gripper_state.data = 1;
                                    gripper_pub.publish(gripper_state);
                                    usleep(400000);
                                    pick = false;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);

                                }else{                      //Place

                                    ROS_WARN("PLACE");
                                    pick = true;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                    usleep(1000000);
                                    gripper_state.data = 0;
                                    gripper_pub.publish(gripper_state);
                                    usleep(400000);
                                    pick = false;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);

                                }

                                count1++;           //MOVED HERE
                                if (count1 == desiredJointsTeachSize){
                                    count1 = 0;
                                }else if (count1 < 0){
                                    count1 = 0;
                                }
//
                            }



                            if (my_plan.trajectory_.joint_trajectory.points.size() != last_trajectory_size){
                                last_trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
                                teachMode_counter=0;
                            }

                            if (teachMode_counter < my_plan.trajectory_.joint_trajectory.points.size()){
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, teachMode_counter);
                                ROS_WARN("message GO! [%d/%d]",teachMode_counter,last_trajectory_size);
                                teachMode_counter++;
                            }else{
                                sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                //ROS_ERROR("message stay!!",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
                            }
                        }else{
                            //ROS_INFO("stopped");
                            if (teachMode_counter < 1) {
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, 999);
                                ROS_ERROR("message stop!! [0/%d]", last_trajectory_size);

                            } else if (teachMode_counter < 15) {
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, teachMode_counter);
                                ROS_ERROR("message stop!! [%d/%d]", teachMode_counter, last_trajectory_size);
                            } else if ((teachMode_counter > 15) && (teachMode_counter < last_trajectory_size - 2)){
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, (teachMode_counter - 10));
                                ROS_ERROR("message stop!! [%d/%d]", (teachMode_counter - 10), last_trajectory_size);
                            }else if (teachMode_counter >= last_trajectory_size -2){
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, last_trajectory_size - 1);
                                ROS_ERROR("message stop!! [%d/%d]", last_trajectory_size - 1, last_trajectory_size);
                            }else{
                                ROS_INFO("something wrong happened with STOP!");
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, 999);
                                ROS_ERROR("message stop!! [0/%d]", last_trajectory_size);
                            }
                            executionOK = true;
                            move_group.stop();
                        }



                    }else {
                        ROS_ERROR("not valid teach mode");
                        //sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);                  //mozno vyriesenie padnutia programu...
                    }

                    ros::spinOnce();
                    loop_rate.sleep();
                }

                break;


                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 5 - TEACH mode HAND                                        /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 5:

                while (ros::ok()){
                    ROS_INFO_ONCE("Teaching mode - torque control ...");

                    //Break while loop when the mode has changed
                    if (current_mode != 5){
                        move_group.stop();
                        count1 = 0;
                        last_trajectory_size = -5;
                        start_state = false;
                        teach_start_state = false;
                        initTeachedPositions = true;
                        teach_mode = 9;
                        zeroPositionForTeach = true;
                        executionOK = true;
                        teachModeHand_counter = 0;
                        help = 0;
                        initMode = false;
                        teachPositionsHandSize = 0;
                        teachPositionsHand.clear();
                        teachPositions.clear();
                        break;
                    }

                    if (initMode){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        gripper_pub.publish(gripper_state);
                    }


                    if (counter1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        counter1 = 0;
                    }
                    counter1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }


                    if (teach_mode == 0){
                        ROS_INFO_ONCE("teaching now");
                        selectedMode.data = 2;
                        mode_pub.publish(selectedMode);
                        if (start_state){     //if teach button was pushed
                            //if (valuesChanged()){
                            ROS_WARN("teached new position !!!!\nJ1=%f J2=%f J3=%f",jointControl_jointValues[0],jointControl_jointValues[1],jointControl_jointValues[2]);
                            teachPositionsHand.push_back(jointControl_jointValues);
                            help =1;
                            start_state = false;
                        }
                        count1 = 0;

                    }else if (teach_mode == 1) {




                        if (initTeachedPositions && (help == 1)) {          //init vector and  calculate IK
                            ROS_WARN("stopped teaching %d",teachPositionsHand.size());
                            for (int i = 0; i < teachPositionsHand.size(); i++) {
                                ROS_INFO("[%d] J1=%f J2=%f J3=%f", i, teachPositionsHand[i][0], teachPositionsHand[i][1],
                                         teachPositionsHand[i][2]);
                            }

                            teachPositionsHandSize = teachPositionsHand.size();
                            if (!(teachPositionsHandSize%2 == 0)){
                                teachPositionsHandSize--;
                                ROS_INFO("size not ok");
                            }else{
                                ROS_INFO("size ok");
                            }
                            sleep(2);
                            for (int i=0; i<teachPositionsHandSize; i++)
                                ROS_WARN("[%d] J1=%f J2=%f J3=%f",i,teachPositionsHand[i][0],teachPositionsHand[i][1],
                                         teachPositionsHand[i][2]);

                            sleep(2);
                            initTeachedPositions = false;


                            ROS_INFO("Current joint poses = %f %f %f",currentJointStates.position[0], currentJointStates.position[1],
                                                                      currentJointStates.position[2]);
                            sleep(2);
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                            selectedMode.data = 6;
                            mode_pub.publish(selectedMode);
                            gripper_state.data = 0;
                            gripper_pub.publish(gripper_state);
                            initMode = true;
                        }



                        if (teach_start_state && !colisionDetection) {

                            ROS_INFO("cout = %d  executionOK = %d",count1,executionOK);

                            if (executionOK) {

                                sleep(2);
                                ROS_WARN("Desired joints : %f %f %f (%d/%d)", teachPositionsHand[count1][0],
                                         teachPositionsHand[count1][1], teachPositionsHand[count1][2],
                                         count1,teachPositionsHandSize-1 );
                                move_group.setJointValueTarget(teachPositionsHand[count1]);
                                jointModeControll(&move_group);
                                last_trajectory_size = -5;
                            }

                            executionOK = inPositionTeach(count1, 2);
                            ROS_INFO("count = %d",count1);


                            if (my_plan.trajectory_.joint_trajectory.points.size() != last_trajectory_size) {
                                last_trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
                                teachMode_counter = 0;
                            }

                            if (executionOK){

                                if (count1 == 0){
                                    ROS_WARN("PICK down");
                                    pick = true;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                    usleep(1000000);
                                    gripper_state.data = 1;
                                    gripper_pub.publish(gripper_state);
                                    usleep(400000);
                                    pick = false;
                                    ROS_WARN("PICK up");
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                }
                                if (count1%2 == 0){         //Pick
                                    ROS_WARN("PICK down");
                                    pick = true;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                    usleep(1000000);
                                    gripper_state.data = 1;
                                    gripper_pub.publish(gripper_state);
                                    usleep(400000);
                                    pick = false;
                                    ROS_WARN("PICK up");
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);

                                }else{                      //Place
                                    ROS_WARN("PLACE down");
                                    pick = true;
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                                    usleep(1000000);
                                    gripper_state.data = 0;
                                    gripper_pub.publish(gripper_state);
                                    usleep(400000);
                                    pick = false;
                                    ROS_WARN("PLACE up");
                                    sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);

                                }

                                count1++;           //MOVED HERE
                                if (count1 == teachPositionsHandSize){
                                    count1 = 0;
                                }else if (count1 < 0){
                                    count1 = 0;
                                }
//
                            }



                            if (teachMode_counter < my_plan.trajectory_.joint_trajectory.points.size()) {
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, teachMode_counter);
                                ROS_WARN("message GO! [%d/%d]", teachMode_counter, last_trajectory_size);
                                teachMode_counter++;
                            } else {
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, last_trajectory_size - 1);
                                ROS_ERROR("message stay!! [%d]", teachMode_counter);
                            }
                        } else {
                            //ROS_INFO("stopped");
                            if (teachMode_counter < 1) {
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, 999);
                                ROS_ERROR("message stop a!! [0/%d]", last_trajectory_size);

                            } else if (teachMode_counter < 15) {
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, teachMode_counter);
                                ROS_ERROR("message stop b!! [%d/%d]", teachMode_counter, last_trajectory_size);
                            } else if ((teachMode_counter > 15) && (teachMode_counter < last_trajectory_size - 2)){
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, (teachMode_counter - 10));
                                ROS_ERROR("message stop c!! [%d/%d]", (teachMode_counter - 10), last_trajectory_size);
                            }else if (teachMode_counter >= last_trajectory_size -2){
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, last_trajectory_size - 1);
                                ROS_ERROR("message stop d!! [%d/%d]", last_trajectory_size - 1, last_trajectory_size);
                            }else{
                                ROS_INFO("something wrong happened with STOP!");
                                sendJointPoses(&pose_pub, &acc_pub, &my_plan, 999);
                                ROS_ERROR("message stop e!! [0/%d]", last_trajectory_size);
                            }
                            executionOK = true;
                            move_group.stop();

                        }
                    }else {
                        ROS_ERROR("not valid teach mode");
                        //sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);          //mozno vyriesenie padnutia programu...........
                        }
                    ros::spinOnce();
                    loop_rate.sleep();
                }

                break;


                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 6 - Moveit mode                                           /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 6:
                while (ros::ok()){

                    ROS_INFO_ONCE("Moveit mode started!");

                    //Break while loop when the mode has changed
                    if (current_mode != 6){
                        count1 = 0;
                        moveitMode.data = false;
                        for (int i=0;i<10;i++){
                            startMoveitMode_pub.publish(moveitMode);
                        }
                        ROS_WARN("Moveit mode stopped!");
                        break;
                    }
                    //
                    selectedMode.data = 6;
                    mode_pub.publish(selectedMode);
                    gripper_pub.publish(gripper_state);

                    //Display end effector pose in GUI
                    if (count1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        count1 = 0;
                    }
                    count1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }

                    //hold position while the movement in moveit isnt started
                    if (!moveitState){
                        sendJointPoses(&pose_pub, &acc_pub, &my_plan, 999);
                    }else{
                        ROS_INFO("Started moving in moveit ! Current joint states: %f %f %f",currentJointStates.position[0],
                                 currentJointStates.position[1], currentJointStates.position[2]);
                    }

                    //Start or continue for get_and_send_planned_path

                    ros::spinOnce();
                    loop_rate.sleep();
                }
                break;


                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 7 - Get info                                              /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 7:
                while (ros::ok()){

                    //Break while loop when the mode has changed
                    if (current_mode != 7){
                        count1 = 0;
                        break;
                    }

                    //Publish selected mode and gripperstate
                    selectedMode.data = 6;
                    mode_pub.publish(selectedMode);
                    gripper_pub.publish(gripper_state);
                    //display end effector pose in GUI
                    if (count1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        count1 = 0;
                    }
                    count1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }


                    //ROS_INFO("Get basic info  ...");
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                break;

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 8 - Set parameters                                         /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 8:
                while (ros::ok()){

                    //Break while loop when the mode has changed
                    if (current_mode != 8){
                        count1 = 0;
                        break;
                    }

                    //Display end effector pose in GUI
                    if (count1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        count1 = 0;
                    }
                    count1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }


                    ROS_INFO("Set params  ...");
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                break;

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
                /////                                     MODE 9 - Colision object                                        /////
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 9:
                while (ros::ok()){

                    //Break while loop when the mode has changed
                    if (current_mode != 9){
                        count1 = 0;
                        break;
                    }

                    //Display end effector pose in GUI
                    if (count1 > 12){
                        sendEndEffectorPose(&actualPose_pub, &move_group);
                        count1 = 0;
                    }
                    count1++;

                    //Central stop - end of program!
                    if (central_stop){
                        selectedMode.data = 6;
                        mode_pub.publish(selectedMode);
                        for (int i=0;i<10;i++)
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("CENTRAL STOP ! PROGRAM END !");
                        return 0;
                    }


                    ROS_INFO("colision object  ...");
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                break;




            default:
                //ROS_INFO("No mode selected!");
                break;
        }

        ros::spinOnce();
        //loop_rate.sleep();
    }





    return 0;
}