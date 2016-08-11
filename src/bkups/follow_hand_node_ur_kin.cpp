#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>


#include <ur_kinematics/ur_kin.h>

#include <math.h>
#include <stdio.h>


using namespace std;
using namespace ur_kinematics;


// Looks up Transfrom from transOrigin to transDestination in transListener and pushes them in transArray
void pushIntoTransformArray(std::string transOrigin, std::string transDestination, std::vector<tf::Transform> * transArray, tf::TransformListener * transListener) {
    tf::StampedTransform transform;
    try{
        // DONE: test order of RightHand / RightForeArm
        // TODO: look up: [ERROR] [1465987288.873699876]: "RightHand" passed to lookupTransform argument target_frame does not exist.
        transListener->lookupTransform(transOrigin, transDestination,
                                  ros::Time(0), transform);
        // WORKED: listener.lookupTransform("/RightForeArm", "/RightHand", ros::Time(0), transform2);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    // convert stampedTransform to Transform datatype
    transArray->push_back( tf::Transform(transform.getRotation(), transform.getOrigin()));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "follow_hand_node");

    ros::NodeHandle node;

    // Interface to UR#
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // TODO: test value 10, implement correct topic
    // TODO: SET THIS VALUE TO /joint_states_ur5 AND LISTEN TO PUBLISHED DATA WITH JOINT_STATE_PUBLISHER
    // JUST CURRENTLY DOESNT WORK WITH NO APPARENT REASON
    // SO FOR NOW JUST PUBLISH JOINTSTATE DIRECTLY TO ROBOT_STATE_PUBLISHER
    ros::Publisher handJointPublisher = node.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // TODO: asses jointNames dynamically
    // maybe with URKinematicsPlugin
    // http://docs.ros.org/hydro/api/ur_kinematics/html/classur__kinematics_1_1URKinematicsPlugin.html
    // #include <ur_moveit_plugin.h>
    // const std::vector<std::string> = ur_kinematics::URKinematicsPlugin kinplugin.getJointNames();
    std::vector<std::string> jointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    double qShoulderPan, qShoulderLift, qElbow, qWrist1, qWrist2, qWrist3;

    tf::Matrix3x3 rotMatrix;
    double* transForKin = new double[16];

    // Perception Neuron Interface
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    tf::TransformListener listener;
    tf::Transform transHipsRightHand; // transform from Hips to RightHand directly
    tf::Transform transBaseLinkRightHand(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)); // transform from base_link to RightHand directly
    tf::StampedTransform stampedTransWorldRightHand;
    tf::TransformBroadcaster broadcaster;

    broadcaster.sendTransform(tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(), "/base_link", "/baseLinkToHand"));

    std::vector<tf::Transform> transformArray;
    // create string array with bodyJoints, TODO: assess all joints && prove if order is correct
    std::vector<std::string> bodyJoints{"Hips","Spine","Spine1","Spine2","Spine3","Neck","RightShoulder","RightArm","RightForeArm", "RightHand"};

    // Ur_kinematics
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    double q[6] = {0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
    double* T = new double[16];
    ur_kinematics::forward(q, T);
    for(int i=0;i<4;i++) {
      for(int j=i*4;j<(i+1)*4;j++)
        printf("%1.3f ", T[j]);
      printf("\n");
    }
    double q_sols[8*6];
    int num_sols;
    num_sols = ur_kinematics::inverse(T, q_sols);
    for(int i=0;i<num_sols;i++)
    printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
       q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
    // useless output. Just output 5 values from zero to 2*M_PI
    for(int i=0;i<=4;i++)
    printf("%f ", M_PI/2.0*i);
    printf("\n");
    // ----------------------------

    // TODO: get frequency at which perc neuron is running and set the appropriate rate
    ros::Rate rate(20.0);
    while (node.ok()){
        // Get Transform / Vector from shoulder_link of ur5 to RightHand of PercNeuron
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

        // fill transformArray with transforms from "Hips" to "RightHand"; i=0 would equal Hips
        for( int i = 0; i < bodyJoints.size()-1; i++){
            pushIntoTransformArray(bodyJoints.at(i), bodyJoints.at(i+1), &transformArray, &listener);
        }

        // get direct vector from Hips to RightHand
        // WORKED: globalTrans = transform.operator*=( transform2.operator*=(transform3) );
        // DANGER: underneath calculation could change values of transforms
        // starts at first element in transform array which should be from Hips to Spine with index 0, then goes along the list
        // TODO: calculate dynamically with transformArraySize!!!!
        // int transArraySize = transformArray.size();
        transHipsRightHand = transformArray.at(0).operator*=(transformArray.at(1).operator*=(
                                transformArray.at(2).operator*=(transformArray.at(3).operator*=(
                                transformArray.at(4).operator*=(transformArray.at(5).operator*=(
                                transformArray.at(6).operator*=(transformArray.at(7).operator*=(
                                transformArray.at(8)))))))));
        transformArray.clear();
        // not necessary: send transform with Hips as parent_node
        // broadcaster.sendTransform(tf::StampedTransform(transHipsRightHand, ros::Time::now(), "/Hips", "/hipsToHand"));


        // Get Transforms from Hips to base_link
        pushIntoTransformArray("WorldPerceptionNeuron", "Hips", &transformArray, &listener);
        pushIntoTransformArray("world", "WorldPerceptionNeuron", &transformArray, &listener);
        pushIntoTransformArray("world", "base_link", &transformArray, &listener);

        // calculate transformation matrix from world to RightHand
        transBaseLinkRightHand = transformArray.at(2).inverseTimes(transformArray.at(1).operator*=(
                        transformArray.at(0).operator*=(transHipsRightHand)));

        transformArray.clear();
        // not necessary
        broadcaster.sendTransform(tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(), "/base_link", "/baseLinkToHand"));
        // ---------------------------------------------------------------------------------

        // form transBaseLinkRightHand into transformation matrix T from ur_kinematics

        // TODO: COMPLETE THIS PROCESS WITH POINTERS!!!
        // is function getColumn(i) starting at 0 or 1?!
        // rotational part of transf. matrix
        for(int i = 0;i<=2;i++){
            // rotational part of transf. matrix
            transForKin[i*4+0] = transBaseLinkRightHand.getBasis().getColumn(i).getX();
            transForKin[i*4+1] = transBaseLinkRightHand.getBasis().getColumn(i).getY();
            transForKin[i*4+2] = transBaseLinkRightHand.getBasis().getColumn(i).getZ();
        }
        // translational part
        transForKin[3] = transBaseLinkRightHand.getOrigin().getX();
        transForKin[7] = transBaseLinkRightHand.getOrigin().getY();
        transForKin[11] = transBaseLinkRightHand.getOrigin().getZ();

        transForKin[12] = 0;
        transForKin[13] = 0;
        transForKin[14] = 0;
        transForKin[15] = 1;
        for(int i=0;i<4;i++) {
          for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", transForKin[j]);
          printf("\n");
        }
        // calculate IK
        num_sols = ur_kinematics::inverse(transForKin, q_sols);
        ROS_INFO_STREAM("number solutions: " << num_sols);
//        for(int i = 0;i<num_sols;i++){
//            printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
//           q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
//        }
        printf("\n\n");
        // turn values over M_PI into negative values to use full range of robot
        // works
        for(int i = 0;i<num_sols;i++){
            for(int j = 0;j<6;j++){
                if(q_sols[i*6+j] > M_PI)
                    q_sols[i*6+j] = q_sols[i*6+j]-2*M_PI;
            }
        }

        // Ur_kinematics
        // inverse() and forward() find kinematics from the base link to the end effector
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//        double q[6] = {M_PI/12, M_PI/12, M_PI/12,M_PI/12 , M_PI/12, M_PI/12};
//        double* T = new double[16];
//        ur_kinematics::forward(q, T);
//        for(int i=0;i<4;i++) {
//          for(int j=i*4;j<(i+1)*4;j++)
//            printf("%1.3f ", T[j]);
//          printf("\n");
//        }
//        double q_sols[8*6];
//        int num_sols;

//        num_sols = ur_kinematics::inverse(T, q_sols);
//        for(int i=0;i<num_sols;i++)
//        printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
//           q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
//        // useless output. Just output 5 values from zero to 2*M_PI
//        for(int i=0;i<=4;i++)
//        printf("%f ", M_PI/2.0*i);
//        printf("\n");

//        // turn values over M_PI into negative values to use full range of robot
//        // works
//        for(int i = 0;i<num_sols;i++){
//            for(int j = 0;j<6;j++){
//                if(q_sols[i*6+j] > M_PI)
//                    q_sols[i*6+j] = q_sols[i*6+j]-2*M_PI;
//            }
//        }
        // ----------------------------
        // Send Data to Robot
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // initial position of ur5 axis in ... direction
        // publish angle to shoulder_pan_joint of joint_states_ur5
        sensor_msgs::JointState joint_msg;

        //Header header, string[] name, float64[] position, float64[] velocity, float64[] effort, joint_msg-header... ros::Time(NOW)

        //joint_msg.header.seq = seq;
        joint_msg.header.stamp = ros::Time::now();
        // TODO: implement correct frame_id
        joint_msg.header.frame_id = "";

        // TODO: assess names dynamically
        joint_msg.name =  jointNames;
        // joint_msg.position = {angle, angleShoulderJoint, -angleElbowJoint, 0.0, 0.0, 0.0};
        int i = 0;
        joint_msg.position = {q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]};
        // joint_msg.position = {M_PI/12, M_PI/12, M_PI/12,M_PI/12 , M_PI/12, M_PI/12};
        joint_msg.velocity = {};
        joint_msg.effort = {};

        handJointPublisher.publish(joint_msg);
        // ----------------------------


//        int i = 0;
//        joint_msg.position = {q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]};
//        handJointPublisher.publish(joint_msg);

        // display all possible joint positions
        // TODO: decide which one we should consider (shoulder up/down, elbow up/down, wrist up/down)
//        int i = 0;
//        while(ros::ok()){
//            if(i == num_sols){
//                i=0;
//            }
//            joint_msg.header.stamp = ros::Time::now();
//            joint_msg.position = {q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]};
//            handJointPublisher.publish(joint_msg);
//            rate.sleep();
//            i++;
//        }
        //joint_msg.position = {qShoulderPan, qShoulderLift, qElbow, qWrist1 , qWrist2, qWrist3};

        //handJointPublisher.publish(joint_msg);

        rate.sleep();
    }

    return 0;
}
