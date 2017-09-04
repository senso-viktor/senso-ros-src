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

    int count1 = 0;
    int numOfPlacePos = 0;
    ros::init(argc, argv, "menu_node");
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7;
    ros::NodeHandle nn1,nn2,nn3,nn4,nn5,nn6,nn7,nn8,nn9,nn10;
    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();


    //interaction with moveit
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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

    //init arm length and offsets for position control
    getOffsets();



    //Publishers
    ros::Publisher actualPose_pub = n1.advertise<geometry_msgs::Pose>("actualPose",1000);
    ros::Publisher errorMessage_pub = n2.advertise<std_msgs::Int32>("errorCode",1000);
    ros::Publisher pose_pub = n3.advertise<geometry_msgs::Pose>("/planned_poses_and_velocities",1000);
    ros::Publisher acc_pub = n4.advertise<geometry_msgs::Point>("/planned_accelerations",1000);
    ros::Publisher mode_pub = n2.advertise<std_msgs::Byte>("/modeSelect",1000);

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
    sleep(2);

    //Publish init data to Matlab
    selectedMode.data = 6;
    for (int j = 0; j < 50; j++){
        sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
        //mode_pub.publish(selectedMode);
        ROS_INFO("Init matlab and scara");
        usleep(50000);
    }

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
                        break;
                    }
                }
            }else{
                //ROS_INFO("Cannot solve IK please enter new positions");
                desiredJointsDEMO[i][0] = 0.0;
                desiredJointsDEMO[i][1] = 0.0;
                desiredJointsDEMO[i][2] = 0.0;
                break;
            }
        }


    }
//    for (int i=0; i<desiredJointsDEMO.size(); i++){
//        ROS_INFO("[%d] J1=%f J2=%f J3=%f",i,desiredJointsDEMO[i][0],desiredJointsDEMO[i][1],desiredJointsDEMO[i][2]);
//    }
    move_group.setPlannerId("RRTConnectkConfigDefault");
    sleep(2);



    //Wait for publishers on specified topics
    waitForPublishers(&scaraJointStates_sub, 1);
    waitForPublishers(&modeSelect_sub, 1);
    waitForPublishers(&startState_sub, 1);
    waitForPublishers(&gripperState_sub, 1);
    waitForPublishers(&jointControl_sub, 1);
    waitForPublishers(&positionControl_sub, 1);
    waitForPublishers(&teachMode_sub, 1);
    waitForPublishers(&teachModeStartState_sub, 1);
    sleep(6);       //wait for matlab init



    //Paralel thread for Colision detection START
    boost::thread fft{forceFeedbackThread};

    while (ros::ok()){
        ROS_INFO_ONCE("while start");

    switch (current_mode){
        ///////////////////////////////////////////////////////////////
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

        //////////////////////////////////////////////////////////////
        case 1:
            while (ros::ok()) {
                //Break while loop when the mode has changed
                if (current_mode != 1){
                    start_state = false;
                    move_group.stop();
                    count1 = 0;
                    jointControl_counter = 0;
                    last_trajectory_size = -5;
                    break;
                }
                //Publish current pose of end effector (for GUI)
                mode_pub.publish(selectedMode);
                if (count1 > 12){
                    sendEndEffectorPose(&actualPose_pub, &move_group);
                    count1 = 0;
                }
                count1++;


                if (start_state && !colisionDetection) {
                    //ROS_INFO_ONCE("Joint control tab started");

                    if (valuesChanged()) {
                        ROS_WARN("Desired joints : %f %f %f", jointControl_jointValues[0], jointControl_jointValues[1],
                                 jointControl_jointValues[2]);
                        jointControl_counter = 0;
                        move_group.setJointValueTarget(jointControl_jointValues);
                        kinematic_state->setJointGroupPositions(joint_model_group, jointControl_jointValues);
                        if (!kinematic_state->satisfiesBounds()) {
                            //ROS_ERROR("Bad input joint values");
                            sendErrorCode(&errorMessage_pub, 1);
                            start_state = false;
                            break;
                        }
                        jointModeControll(&move_group);
                    }
                    if (my_plan.trajectory_.joint_trajectory.points.size() != last_trajectory_size){
                        last_trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
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
                    //ROS_ERROR("Movement stopped!!");
                    //ROS_INFO("size of plan %d", my_plan.trajectory_.joint_trajectory.points.size());
                    if (jointControl_counter < 1){
                        sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("message stop!! [0/%d]",last_trajectory_size);
                    }else if (jointControl_counter < 15){
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



        //////////////////////////////////////////////////////////////////////////////
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
                    break;
                }
                //Publish current pose of end effector (for GUI)
                mode_pub.publish(selectedMode);
                if (count1 > 12){
                    sendEndEffectorPose(&actualPose_pub, &move_group);
                    count1 = 0;
                }
                count1++;

                if (start_state && !colisionDetection) {
                    //ROS_INFO("Position control start request");
                    if (positionsChanged()){
                        while (ros::ok()){
                            if (calculateIK(positionControl_values[0], positionControl_values[1], positionControl_values[2], IK_mode, 0, 0)){
                                move_group.setJointValueTarget(joint_positions);
                                kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
                                if (kinematic_state->satisfiesBounds()){
                                    IK_mode = 1;
                                    success = move_group.plan(my_plan);
                                    if (success){
                                        ROS_INFO("Succesful plan!.. now moving to place");
                                        move_group.asyncExecute(my_plan);
                                        if (my_plan.trajectory_.joint_trajectory.points.size() != last_trajectory_size){
                                            last_trajectory_size = my_plan.trajectory_.joint_trajectory.points.size();
                                            positionControl_counter=0;
                                        }
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
                                        break;
                                    }
                                }
                            }else{
                                ROS_ERROR("No solution found");
                                ROS_INFO("Cannot solve IK please enter new positions");
                                sendErrorCode(&errorMessage_pub, 5);
                                start_state = false;
                                break;
                            }
                        }

                     }

//
                    if (positionControl_counter < my_plan.trajectory_.joint_trajectory.points.size()){
                        sendJointPoses(&pose_pub,&acc_pub, &my_plan, positionControl_counter);
                        ROS_WARN("message GO! [%d/%d]",positionControl_counter,last_trajectory_size);
                        positionControl_counter++;
                    }else{
                        sendJointPoses(&pose_pub,&acc_pub, &my_plan, last_trajectory_size-1);
                        ROS_ERROR("message stay!!",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
                    }
                }else {
                    //ROS_ERROR("Movement stopped!!");

                    if (positionControl_counter < 1){
                        sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                        ROS_ERROR("message stop!! [0/%d]",last_trajectory_size);
                    }else if (positionControl_counter < 15){
                        sendJointPoses(&pose_pub,&acc_pub, &my_plan, jointControl_counter);
                        ROS_ERROR("message stop!! [%d/%d]", positionControl_counter-10,last_trajectory_size);
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


        //////////////////////////////////////////////////////////////////////
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
                    //executionOK = true;
                    break;
                }
                mode_pub.publish(selectedMode);
                //Publish current pose of end effector (for GUI)
                if (count1 > 12){
                    sendEndEffectorPose(&actualPose_pub, &move_group);
                    count1 = 0;
                }
                count1++;

                if (start_state){       //colision!!!!!

                    ROS_INFO("running");
                    executionOK = inPosition(DEMO_mode);

                    if (executionOK){

                        sleep(2);
                        DEMO_mode++;
                        if (DEMO_mode > 3){
                            DEMO_mode = 0;  //home position
                        }
                        if (DEMO_mode == 3){    //for placing positions
                            if (numOfPlacePos == 7)
                                numOfPlacePos = 0;
                            DEMO_mode += numOfPlacePos;
                            numOfPlacePos++;

                        }
                        ROS_WARN("Desired joints : %f %f %f", desiredJointsDEMO[DEMO_mode][0], desiredJointsDEMO[DEMO_mode][1],
                                 desiredJointsDEMO[DEMO_mode][2]);
                        demoControl_counter = 0;
                        move_group.setJointValueTarget(desiredJointsDEMO[DEMO_mode]);
                        jointModeControll(&move_group);
                    }

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

                ROS_INFO("Current mode = %d , num of place position = %d",DEMO_mode,numOfPlacePos);
                if (DEMO_mode == 3){
                    ROS_INFO("current index %d",DEMO_mode+numOfPlacePos);
                }

                ros::spinOnce();
                loop_rate.sleep();
            }
            break;


        ////////////////////////////////////////////////////////////////
        case 4:
            while (ros::ok()){
                ROS_INFO_ONCE("teaching mode");

                //Break while loop when the mode has changed
                if (current_mode != 4){
                    move_group.stop();
                    count1 = 0;
                    last_trajectory_size = -5;
                    start_state = false;
                    initTeachedPositions = true;
                    IK_mode = 1;
                    zeroPositionForTeach = true;
                    teachMode_counter = 0;
                    //teachPositions.clear();
                    break;
                }

                if (zeroPositionForTeach){      //The first element in vector is [0 0 0]
                    teachPositions.push_back(desiredPositionsDEMO[0]);
                    zeroPositionForTeach = false;
                }

                if (teach_mode == 0){           //teach mode

                    //Set mode for torque control and publish
                    //selectedMode.data = 2;
                    //mode_pub.publish(selectedMode);

                    //desiredJointsTeach.clear();
                    ROS_INFO_ONCE("teaching now");
                    if (start_state){     //if teach button was pushed
                        if (teachPointChanged()){
                            ROS_WARN("teached new positions!!!!");
                            teachPositions.push_back(currentTeachPoint);
                            start_state = false;
                        }

                    }


                }else if (teach_mode == 1)   {                      //run mode

                    //Set mode for joint control
                    selectedMode.data = 6;
                    mode_pub.publish(selectedMode);

                    if (initTeachedPositions){          //init vector and  calculate IK
                        ROS_INFO("stopped teaching");
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

                        for (int i=0; i<desiredJointsTeach.size(); i++){
                            ROS_INFO("[%d] J1=%f J2=%f J3=%f",i,desiredJointsTeach[i][0],desiredJointsTeach[i][1],desiredJointsTeach[i][2]);
                        }
                        initTeachedPositions = false;
                    }


                    if (teach_start_state && !colisionDetection){

                        if (count1 != -1){
                            executionOK = inPositionTeach(count1);
                        }else {
                            executionOK = true;
                        }

                        if (executionOK){
                            sleep(2);
                            count1++;
                            if (count1 == desiredJointsTeach.size()){
                                count1 = 0;
                            }

                            ROS_WARN("Desired joints : %f %f %f", desiredJointsTeach[count1][0], desiredJointsTeach[count1][1],
                                     desiredJointsTeach[count1][2]);
                            move_group.setJointValueTarget(desiredJointsTeach[count1]);
                            jointModeControll(&move_group);
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
                            ROS_ERROR("message stay!!",pos_and_vel.position.x, pos_and_vel.position.y, pos_and_vel.position.z);
                        }
                    }else{
                        ROS_INFO("stopped");
                        if (teachMode_counter < 1){
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, 999);
                            ROS_ERROR("message stop!! [0/%d]",last_trajectory_size);

                        }else if (teachMode_counter < 15){
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, teachMode_counter);
                            ROS_ERROR("message stop!! [%d/%d]", demoControl_counter-10,last_trajectory_size);
                        }else{
                            sendJointPoses(&pose_pub,&acc_pub, &my_plan, (teachMode_counter-10));
                            ROS_ERROR("message stop!! [%d/%d]", (teachMode_counter-10),last_trajectory_size);
                        }
                        count1 = -1;
                        move_group.stop();
                    }

                }else {
                    ROS_ERROR("not valid teach mode");
                }

                ros::spinOnce();
                loop_rate.sleep();
            }

            break;


        ////////////////////////////////////////////////////////////////
        case 5:
            while (ros::ok()){
                ROS_INFO_ONCE("Teaching mode - torque control ...");

                //Break while loop when the mode has changed
                if (current_mode != 5){
                    move_group.stop();
                    count1 = 0;
                    last_trajectory_size = -5;
                    start_state = false;
                    break;
                }



                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 6:
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 6)
                    break;

                ROS_INFO("Set parameters tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 7:         //Ked budem dorabat veci do GUI
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
        //loop_rate.sleep();
    }





    return 0;
}