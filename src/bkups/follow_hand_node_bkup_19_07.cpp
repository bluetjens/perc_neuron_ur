#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>

// /home/perceptionneuron/ur_ws/src/universal_robot/ur_kinematics/include/ur_kinematics
// /home/perceptionneuron/ur5_perc_test_ws/src/follow_hand/src
// cd ../../../../ur_ws/src/universal_robot/ur_kinematics/include/ur_kinematics/ur_kin.h

#include <math.h>
#include <stdio.h>

#include <ctime> // runtime measurement

#include <qt4/Qt/qvector.h>

#include <tumtools/Math/ButterFilter2.h>
#include <tumtools/Math/MathTools.h>
#include <tumtools/Math/EigenDefs.h>
#include <tumtools/Math/MathDefs.h>


using namespace std;
#include <ur_kinematics/ur_kin.h>
// using namespace ur_kinematics;


// TODO: DECLARE ALL MAGIC NUMBERS AT HEAD OF CODE // DONE
// TODO: MAKE IT EASIER TO INITALLY FETCH THE ROBOT (rotation)
// TODO: SOMEHOW MAKE LASTPOSITION THE DEFAULT / START / INITIAL POSITION VECTOR
int initializeRobot(ros::NodeHandle * node, std::vector<double> * robotDefaultPos, std::vector<double> * percNeuronPos){

}

// Get Transform / Vector from base_link of ur5 to RightHand of PercNeuron
//int getTransBaseLinkRightHand(bodyJoints, transformArray, listener, , ){
//    transHipsRightHand

//    // Get Transform / Vector from shoulder_link of ur5 to RightHand of PercNeuron
//    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//    // fill transformArray with transforms from "Hips" to "RightHand"; i=0 would equal Hips
//    /**     table for solutions:
////                 shoulder left = on left side of body (from eyes of perception neuron person) and shoulder_pan_joint> 0?
////                 i = 0 : elbow up;    shoulder left;      wrist up / away from body
////                 i = 1 : elbow down;  shoulder left;      wrist up / away from body
////                 i = 2 : elbow up;    shoulder left;      wrist down / at body;       -0.713767 -0.823703 1.703598 2.607962 -1.540564 -2.063133
////                 i = 3 : elbow down;  shoulder left;      wrist down / at body
////                 i = 4 : elbow down;  shoulder right;     wrist down / at body
////                 i = 5 : elbow up;    shoulder right;     wrist at down / body;       2.805665 -2.314047 -1.709266 0.515700 1.246372 -1.930651
////                 i = 6 : elbow down;  shoulder right;     wrist up / away from body
////                 i = 7 : elbow up;    shoulder right;     wrist up / away from body
//    */     for( int i = 0; i < bodyJoints.size()-1; i++){
//        pushIntoTransformArray(bodyJoints.at(i), bodyJoints.at(i+1), &transformArray, &listener);
//    }

//    // get direct vector from Hips to RightHand
//    // WORKED: globalTrans = transform.operator*=( transform2.operator*=(transform3) );
//    // DANGER: underneath calculation could change values of transforms
//    // starts at first element in transform array which should be from Hips to Spine with index 0, then goes along the list
//    // TODO: calculate dynamically with transformArraySize!!!!
//    // int transArraySize = transformArray.size();
//    transHipsRightHand = transformArray.at(0).operator*=(transformArray.at(1).operator*=(
//                            transformArray.at(2).operator*=(transformArray.at(3).operator*=(
//                            transformArray.at(4).operator*=(transformArray.at(5).operator*=(
//                            transformArray.at(6).operator*=(transformArray.at(7).operator*=(
//                            transformArray.at(8)))))))));
//    transformArray.clear();
//    // not necessary: send transform with Hips as parent_node
//    broadcaster.sendTransform(tf::StampedTransform(transHipsRightHand, ros::Time::now(), "/Hips", "/hipsToHand"));
//    // ---------------------------------------------------------------------------------

//    // Get correct solution for Transform forward kinematics as transformation
//    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

///*      // Get Transforms from base_link to end effector
////        pushIntoTransformArray("wrist_3_link", "ee_link", &transformArray, &listener);
////        pushIntoTransformArray("wrist_2_link", "wrist_3_link", &transformArray, &listener);
////        pushIntoTransformArray("wrist_1_link", "wrist_2_link", &transformArray, &listener);
////        pushIntoTransformArray("forearm_link", "wrist_1_link", &transformArray, &listener);
////        pushIntoTransformArray("upper_arm_link", "forearm_link", &transformArray, &listener);
////        pushIntoTransformArray("shoulder_link", "upper_arm_link", &transformArray, &listener);
////        pushIntoTransformArray("base_link", "shoulder_link", &transformArray, &listener);


////        // calculate transformation matrix from base_link to ee_link
////        transBaseLinkEELink = transformArray.at(0).operator*=(transformArray.at(1).operator*=(
////                                transformArray.at(2).operator*=(transformArray.at(3).operator*=(
////                                transformArray.at(4).operator*=(transformArray.at(5).operator*=(
////                                transformArray.at(6)))))));

////        transformArray.clear();
////        broadcaster.form(tf::StampedTransform(transBaseLinkEELink, ros::Time::now(), "/base_link", "/baseToEE"));
////*/

//    // Calculate transformation matrix from base_link to RightHand
//    pushIntoTransformArray("/base_link", "/world", &transformArray, &listener);
//    pushIntoTransformArray("/world", "/WorldPerceptionNeuron", &transformArray, &listener);
//    pushIntoTransformArray("/WorldPerceptionNeuron", "/Hips", &transformArray, &listener);
///*        // Uncomment if vector /hipsToHand is published externally
//    pushIntoTransformArray("/Hips", "/hipsToHand", &transformArray, &listener);
//    transBaseLinkRightHand = transformArray.at(0).inverseTimes(transformArray.at(1).operator*=(transformArray.at(2).operator*=(
//                                transformArray.at(3))));
//*/
//    // Calculate transformation matrix from world to RightHand
//    transBaseLinkRightHand = transformArray.at(0).inverseTimes(transformArray.at(1).operator*=(transformArray.at(2).operator*=(
//                                transHipsRightHand)));
//    transformArray.clear();
//    //broadcaster.sendTransform(tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(), "/base_link", "/baseLinkToHand"));

//    // Rotate goal Transform M_PI around y to turn end effector into the direction of the hand/palm
//    transBaseLinkRightHand = transBaseLinkRightHand.operator*(rotate);
//    broadcaster.sendTransform(tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(), "/base_link", "/baseLinkToHand"));

//}

void calculateTrajectory(ros::NodeHandle * node, std::vector<double> * goalPosition, std::vector<double> * lastPosition, sensor_msgs::JointState * jointMsg, ros::Publisher * handJointPublisher, double * framesPerSecPercNeuron){
    // Initialize Variables
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
    bool goalPossible = false;
    // TODO: goalViolation is for testing purposes; in real: return after goalPossible = false!
    // therefore, eliminate goalViolation from code and just use goalPossible
    bool goalViolation = false;
    // TODO: check if default value is ok!!!
    // -2*M_PI < deltaPosition < 2*M_PI
    std::vector<double> deltaPosition = {0,0,0,0,0,0};
    std::vector<double> publishPosition = {0,0,0,0,0,0};

    // Parameters for velocity control
    // default maxVel = 17° / sec = M_PI*17/180 rad/sec = 0.29670597283 rad/sec
    // we have for the hole movement 40ms time from now on. (Perception Neuron is running at 20Hz)
    double maxVel = 17; // in degree/sec
    double publishRate = 0.008; // 125 Hz
    double timeForMainLoop = 0.003;
    double loopTime = (1/ *framesPerSecPercNeuron) - timeForMainLoop;
    int numberPublishes = floor(loopTime / publishRate); // round down no never publish faster than 125 Hz
    double tempDelta = 0; // temporary deltaPosition for velocity control

    int elbowJointIndex = 2; // Index of elbow joint in goal/solution/delta/lastPosition vectors
/*  // print solution vector:
    printf("\nSolution Vector: ");
    for (int i = 0; i < goalPosition->size(); i++){
        printf(" %f", goalPosition->at(i));
    }
   printf("\n");
 //*/

    // position i: 0:qShoulderPan, 1:qShoulderLift, 2:qElbow, 3:qWrist1, 4:qWrist2, 5:qWrist3;

    // Check reachability of goals with elbow joint restriction:
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // TODO: break if goalPosition is not reachable
    // possible default range: 20° - 160° = PI/9 - 8/9*PI
    double qElbowMin = M_PI/9;
    double qElbowMax = M_PI*8/9;
    node->getParam("/follow_hand/qElbowMin", qElbowMin);
    node->getParam("/follow_hand/qElbowMax", qElbowMax);
    // ROS_INFO_STREAM("qElbowMin: "<< qElbowMin);
    // Works safe for solution 5: elbow up; shoulder right; wrist down / at body
    if(qElbowMin < M_PI-goalPosition->at(elbowJointIndex) && M_PI-goalPosition->at(elbowJointIndex) < qElbowMax)
        goalPossible = true;
    else if(qElbowMin < M_PI+goalPosition->at(elbowJointIndex) && M_PI+goalPosition->at(elbowJointIndex) < qElbowMax){ // Negative joint value
        goalPossible = true;
    }
    else{
        goalPossible = false;
        goalViolation = true;
        printf("\n Desired goal is outside of joint angle boundaries, wait for User to come back to robot.");
        return;
    }


    // Differenz goalPosition / lastPosition bilden
    for(int i = 0; i < goalPosition->size(); i++){
        deltaPosition.at(i) = goalPosition->at(i) - lastPosition->at(i);
    }
    //* // print lastPosition vector:
        printf("\nlastPosition Vector: ");
        for (int i = 0; i < lastPosition->size(); i++){
            printf(" %f", lastPosition->at(i));
        }
    //*/
    //* // print goal vector:
        printf("\ngoal Vector: ");
        for (int i = 0; i < goalPosition->size(); i++){
            printf(" %f", goalPosition->at(i));
        }
    //*/
    //* // print delta vector:
        printf("\ndelta Vector: ");
        for (int i = 0; i < deltaPosition.size(); i++){
            printf(" %f", deltaPosition.at(i));
        }
    //*/



    // Check for max velocity regulation
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    node->getParam("/follow_hand/maxVel", maxVel);
    maxVel = M_PI/180 * maxVel; // convert deg to rad
    for (int i = 0; i < deltaPosition.size(); i++){
        //do{
            if(deltaPosition.at(i) < -M_PI){
                double checkVel = deltaPosition.at(i) + 2*M_PI;
                if(checkVel/loopTime < maxVel) // maxVel not desired
                    goalPossible = true;
                else{
                    goalPossible = false;
                    //return;
                    goalViolation = true;
                    printf("\n1:gehalten an Gelenk i= %d maxVel = %f, desiredVel = %f", i, maxVel, deltaPosition.at(i)/loopTime);
                    //printf("\n desired vel bigger velMax, wait for User to come back to robot.");
                    return;
                    // sleep(1);
                    // TODO: TEST THIS ! (should work but 17° /sec is super slow
                    // TODO: LET ROBOT PAUSE UNTIL USER RETURNS TO ROBOT
                }
            }
            else if(M_PI < deltaPosition.at(i)){
                double checkVel = deltaPosition.at(i) - 2*M_PI;
                if(-maxVel < checkVel/loopTime) // maxVel not desired
                    goalPossible = true;
                else{
                    goalPossible = false;
                    //return;
                    goalViolation = true;
                    printf("\n2:gehalten an Gelenk i= %d maxVel = %f, desiredVel = %f", i, maxVel, deltaPosition.at(i)/loopTime);
                    //printf("\n desired vel bigger velMax, wait for User to come back to robot.");
                    return;
                    // sleep(1);
                    // TODO: TEST THIS ! (should work but 17° /sec is super slow
                    // TODO: LET ROBOT PAUSE UNTIL USER RETURNS TO ROBOT
                }
            }
            // TODO CHECK NEGATIVE VALUES
            else if(abs(deltaPosition.at(i))/loopTime < maxVel) // maxVel not desired
                goalPossible = true;
            else{
                goalPossible = false;
                //return;
                goalViolation = true;
                printf("\n3:gehalten an Gelenk i= %d maxVel = %f, desiredVel = %f", i, maxVel, deltaPosition.at(i)/loopTime);
                //printf("\n desired vel bigger velMax, wait for User to come back to robot.");
                return;
                // sleep(1);
                // TODO: TEST THIS ! (should work but 17° /sec is super slow
                // TODO: LET ROBOT PAUSE UNTIL USER RETURNS TO ROBOT
            }
        //}
        //while(goalPossible = false);
    }
    if(goalViolation != 0){
        sleep(10);
    }
    //printf("\nsolution %s", goalPossible ? "possible" : "not possible");

    // Compare interpolates solution to normal
    // TODO: TROUGH COMPARISON OUT OF CODE
    bool interpolate = 1;
    node->getParam("/follow_hand/interpolate", interpolate);
    if(interpolate){
        // Interpolate and publish solutions
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // all solutions from here on should we realisable and not damaging the robot
        // interpolate in loopTime ms to send in a rate of publishRate (default 8ms)
        // TODO: evaluate cycle Time from last published position to first one. (should be 50ms - 40ms - ~3ms)
        ros::Rate rate(1/publishRate); //should be 125 Hz = 1/8ms
        ros::Time begin = ros::Time::now();
        ros::Duration elapsedTime = begin - begin;

        for (int step = 1; step <= numberPublishes; step++){ //numberPublishes = 5
            // Interpolate und set publishPosition
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            for (int i = 0; i < deltaPosition.size(); i++){
                // Check for jump from positive to negative value or vize versa, caused by negating values after ik calculation in main
                // So keeps boundaries -M_PI < publishPosition < M_PI
                if(M_PI < deltaPosition.at(i)){
                    tempDelta = deltaPosition.at(i)-2*M_PI;
                    publishPosition.at(i) = lastPosition->at(i) + ((float)step/(float)numberPublishes)*tempDelta;
                    if(publishPosition.at(i) < -M_PI){
                        publishPosition.at(i) += 2*M_PI;
                    }
                    printf("\njump from negative to positive, publishPosition: %f, lastPos %f, deltaPos %f", publishPosition.at(i), lastPosition->at(i), deltaPosition.at(i));
                    ROS_DEBUG_STREAM("Detected M_PI < (jump from negative to positive desired joint angle)");
                }
                else if(deltaPosition.at(i) < -M_PI){
                    // TODO: declare at beginning
                    tempDelta = deltaPosition.at(i) + 2*M_PI;
                    publishPosition.at(i) = lastPosition->at(i) + ((float)step/(float)numberPublishes)*tempDelta;
                    if(M_PI < publishPosition.at(i)){
                        publishPosition.at(i) -= 2*M_PI;
                    }
                    ROS_DEBUG_STREAM("Detected (jump from positive to negative desired joint angle) < -M_PI");
                    printf("\njump pos to negative detected.");
                }
                else
                    // printf("\ninside else statemetn");
                    publishPosition.at(i) = lastPosition->at(i) + ((float)step/(float)numberPublishes)*deltaPosition.at(i);
            }
            // Send / Publish Data to Robot Controller
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            jointMsg->header.stamp = ros::Time::now();
            jointMsg->position = {publishPosition.at(0), publishPosition.at(1), publishPosition.at(2), publishPosition.at(3), publishPosition.at(4), publishPosition.at(5)};
            // jointMsg.velocity = {};
            //* // print published position:
                printf("\npublished Position at step = %d: ", step);
                for (int i = 0; i < publishPosition.size(); i++){
                    printf(" %f", publishPosition.at(i));
                }
            //*/
            handJointPublisher->publish(*jointMsg);
            // sleep(5);
            rate.sleep(); // TODO test if cycle really is 8ms
    /**     // measure cycle time, should equal publishRate
            elapsedTime = ros::Time::now() - begin;
            printf("/n");
            ROS_INFO_STREAM("elapsed time: "<< elapsedTime);
            begin = ros::Time::now();
    //*/
        }
        // copy last published Position to lastPosition
        for (int i = 0; i < lastPosition->size(); i++)
            lastPosition->at(i) = publishPosition.at(i);
        // ----------------------------
        for (int i = 0; i < lastPosition->size(); i++){
            if(publishPosition.at(i) < -M_PI || M_PI < publishPosition.at(i)){
                printf("\npublishPosition value: %f at i = %d ", publishPosition.at(i), i);
                sleep(5);
            }
        }


    }
    else{
        for (int i = 0; i < goalPosition->size(); i++)
            publishPosition.at(i) = goalPosition->at(i);
        jointMsg->header.stamp = ros::Time::now();
        jointMsg->position = {publishPosition.at(0), publishPosition.at(1), publishPosition.at(2), publishPosition.at(3), publishPosition.at(4), publishPosition.at(5)};
        handJointPublisher->publish(*jointMsg);

    }

    // NEXT STEPS: TEST WITH PN = record pick and place data, test 17° /sec
    // somehow get initial position of robot. (maybe from robot_state_publisher)
    // TEST WITH REAL ROBOT

}

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

// Only works for joint_limited version!, if other is desired: change calculateTraj.
int main(int argc, char** argv){
    ros::init(argc, argv, "follow_hand_node");

    ros::NodeHandle node;
    int counter = 0;
    int i = 0;
    std::vector<int> solutionIndices;

    // static VVectorDOFd getJointPVT5(const VectorDOFd& start, const VectorDOFd& goal, double t_current, double t_total);
    QVector<double> doubleVector;
    Eigen::Matrix<double, 6, 1>  startvectordofd; // Matrix< _Scalar, _Rows, _Cols >
    startvectordofd << 0, 0, 0, 0, 0, 0;


    Eigen::Matrix<double, 6, 1>  goalvectordofd;
    //vectordofdgoal << 2.809080, -2.326490, -1.572308, 0.347548, 1.137317, -1.876026;
    goalvectordofd << 1, 1, 1, 1, 1, 1;
    std::cout << "goal: " << goalvectordofd << std::endl;

    printf("\nvectordofd initialized");
    // Tum::VVectorDOFd vvectordofd2;
    QVector<Tum::VectorDOFd> interpolationvvectordofd;
    interpolationvvectordofd = Tum::Tools::MathTools::getJointPVT5(startvectordofd, goalvectordofd, 0.00, 1.10);
    printf("\nto calculated getjointpvt5");
    printf("\nprint getjointpvt5: ");
    // vvectordofd[0] = vectordofdgoal;
    // interpolationvvectordofd.append(goalvectordofd);
    for (int i = 0; i < interpolationvvectordofd.size(); ++i) {
        std::cout << interpolationvvectordofd[i] << std::endl;
    }
        //sleep(1);
    // Set FrameRate to scan data; PercNeuron (32 Neuron Version) runs at 60Hz, according to online datasheet
    // If number of neurons of connected suit is less than 19 the acquisition frequency / frequency of callback function is 120Hz
    double framesPerSecPercNeuron = 60;
    // TODO: share Parameter with rosserial_server so that windows machine can read the value!
    //node.setParam("/follow_hand/framesPerSecPercNeuron", framesPerSecPercNeuron);


    // Interface to UR#
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // all ur5 values, including DH (Denavit-Hartenberg) values are stored on the parameter server
    // on $ rosparam get /robot_description

    // TODO: test value 10, implement correct topic
    // TODO: SET THIS VALUE TO /joint_states_ur5 AND LISTEN TO PUBLISHED DATA WITH JOINT_STATE_PUBLISHER
    // JUST CURRENTLY DOESNT WORK WITH NO APPARENT REASON
    // SO FOR NOW JUST PUBLISH JOINTSTATE DIRECTLY TO ROBOT_STATE_PUBLISHER
    ros::Publisher handJointPublisher = node.advertise<sensor_msgs::JointState>("/follow_hand/joint_states_ur5", 10);

    // TODO: asses jointNames dynamically
    // maybe with URKinematicsPlugin
    // http://docs.ros.org/hydro/api/ur_kinematics/html/classur__kinematics_1_1URKinematicsPlugin.html
    // #include <ur_moveit_plugin.h>
    // const std::vector<std::string> = ur_kinematics::URKinematicsPlugin kinplugin.getJointNames();
    std::vector<std::string> jointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    double qShoulderPan, qShoulderLift, qElbow, qWrist1, qWrist2, qWrist3;

    tf::Matrix3x3 rotMatrix;
    double* transForKin = new double[16]; // 4x4 transform matrix
    int numSols;
    double qSolsArray[8*6]; // 8 possible ik solutions * 6 joints
    std::vector<double> ikSolution{0,0,0,0,0,0};
    /**     table for solutions:
//                 shoulder left = on left side of body (from eyes of perception neuron person) and shoulder_pan_joint> 0?
//                 i = 0 : elbow up;    shoulder left;      wrist up / away from body
//                 i = 1 : elbow down;  shoulder left;      wrist up / away from body
//                 i = 2 : elbow up;    shoulder left;      wrist down / at body;       -0.713767 -0.823703 1.703598 2.607962 -1.540564 -2.063133
//                 i = 3 : elbow down;  shoulder left;      wrist down / at body
//                 i = 4 : elbow down;  shoulder right;     wrist down / at body
//                 i = 5 : elbow up;    shoulder right;     wrist at down / body;       2.805665 -2.314047 -1.709266 0.515700 1.246372 -1.930651
//                 i = 6 : elbow down;  shoulder right;     wrist up / away from body
//                 i = 7 : elbow up;    shoulder right;     wrist up / away from body
    */
    int ikSolNr = 5; // See above part "table for solutions" for reason
    //TODO: THIS HAS TO BE DEFAULT STATE!!!!!!!
    // so user has to fetch robot by taking his arm into the default state
    // TODO: GET DEFAULT STATE !!!
    // or: drive from any state into default state / but therefore first state has to be known
    std::vector<double> lastPosition{0,0,0,0,0,0};
    //std::vector<double> lastPosition{2.809080, -2.326490, -1.572308, 0.347548, 1.137317, -1.876026};



    // Initialize message to publish joint states
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // initial position of ur5 axis in ... direction
    // publish angle to shoulder_pan_joint of joint_states_ur5
    // Header header, string[] name, float64[] position, float64[] velocity, float64[] effort, jointMsg-header... ros::Time(NOW)
    sensor_msgs::JointState jointMsg;
    // jointMsg.header.seq = seq;
    jointMsg.header.stamp = ros::Time::now();
    // TODO: implement correct frame_id
    // TODO: set correct velocity and effort
    jointMsg.header.frame_id = "";
    jointMsg.name =  jointNames;
    jointMsg.velocity = {};
    jointMsg.effort = {};


    // Perception Neuron Interface
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    tf::TransformListener listener;
    tf::Transform transHipsRightHand; // transform from Hips to RightHand directly
    tf::Transform transBaseLinkRightHand(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)); // transform from base_link to RightHand directly
    tf::StampedTransform stampedTransWorldRightHand;
    tf::Transform transBaseLinkEELink;
    tf::TransformBroadcaster broadcaster;
    tf::Transform rotate(tf::Quaternion(tf::Vector3(0,1,0), M_PI), tf::Vector3(0,0,0));

    broadcaster.sendTransform(tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(), "/base_link", "/baseLinkToHand"));

    std::vector<tf::Transform> transformArray;
    // create string array with bodyJoints, TODO: assess all joints && prove if order is correct
    std::vector<std::string> bodyJoints{"Hips","Spine","Spine1","Spine2","Spine3","Neck","RightShoulder","RightArm","RightForeArm", "RightHand"};



/** // Test: Ur_kinematics package
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    double q[6] = {0.0, 0.0, 1.0, 0.0, 1.0, 0.0}; // solution array
    double* T = new double[16];

    ur_kinematics::forward(q, T);
    for(int i=0;i<4;i++) {
      for(int j=i*4;j<(i+1)*4;j++)
        printf("%1.3f ", T[j]);
      printf("\n");
    }
    double q_sols[8*6]; // max number of solutions = 8; number of joints of ur# is 6
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
*/
    // setup time to get perc_neuron tf_broadcaster started
    sleep(2);
    // TODO: get frequency at which perc neuron is running and set the appropriate rate
    ros::Time begin = ros::Time::now();
    ros::Duration elapsedTime = begin - begin;
    ros::Rate rate(framesPerSecPercNeuron);
    //ros::Rate rate(20.0);
    while (node.ok()){
        //begin = ros::Time::now();
/*      // measure elapsed Time: is about 0.05000 +- 0.0001 secs
        elapsedTime = ros::Time::now() - begin;
        begin = ros::Time::now();
        ROS_INFO_STREAM("elapsed time: " << elapsedTime);
//*/

        // Get Transform / Vector from shoulder_link of ur5 to RightHand of PercNeuron
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // fill transformArray with transforms from "Hips" to "RightHand"; i=0 would equal Hips
        /**     table for solutions:
//                 shoulder left = on left side of body (from eyes of perception neuron person) and shoulder_pan_joint> 0?
//                 i = 0 : elbow up;    shoulder left;      wrist up / away from body
//                 i = 1 : elbow down;  shoulder left;      wrist up / away from body
//                 i = 2 : elbow up;    shoulder left;      wrist down / at body;       -0.713767 -0.823703 1.703598 2.607962 -1.540564 -2.063133
//                 i = 3 : elbow down;  shoulder left;      wrist down / at body
//                 i = 4 : elbow down;  shoulder right;     wrist down / at body
//                 i = 5 : elbow up;    shoulder right;     wrist at down / body;       2.805665 -2.314047 -1.709266 0.515700 1.246372 -1.930651
//                 i = 6 : elbow down;  shoulder right;     wrist up / away from body
//                 i = 7 : elbow up;    shoulder right;     wrist up / away from body
        */     for( int i = 0; i < bodyJoints.size()-1; i++){
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
        broadcaster.sendTransform(tf::StampedTransform(transHipsRightHand, ros::Time::now(), "/Hips", "/hipsToHand"));
        // ---------------------------------------------------------------------------------

        // Get correct solution for Transform forward kinematics as transformation
        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

/*      // Get Transforms from base_link to end effector
//        pushIntoTransformArray("wrist_3_link", "ee_link", &transformArray, &listener);
//        pushIntoTransformArray("wrist_2_link", "wrist_3_link", &transformArray, &listener);
//        pushIntoTransformArray("wrist_1_link", "wrist_2_link", &transformArray, &listener);
//        pushIntoTransformArray("forearm_link", "wrist_1_link", &transformArray, &listener);
//        pushIntoTransformArray("upper_arm_link", "forearm_link", &transformArray, &listener);
//        pushIntoTransformArray("shoulder_link", "upper_arm_link", &transformArray, &listener);
//        pushIntoTransformArray("base_link", "shoulder_link", &transformArray, &listener);


//        // calculate transformation matrix from base_link to ee_link
//        transBaseLinkEELink = transformArray.at(0).operator*=(transformArray.at(1).operator*=(
//                                transformArray.at(2).operator*=(transformArray.at(3).operator*=(
//                                transformArray.at(4).operator*=(transformArray.at(5).operator*=(
//                                transformArray.at(6)))))));

//        transformArray.clear();
//        broadcaster.form(tf::StampedTransform(transBaseLinkEELink, ros::Time::now(), "/base_link", "/baseToEE"));
//*/

        // Calculate transformation matrix from base_link to RightHand
        pushIntoTransformArray("/base_link", "/world", &transformArray, &listener);
        pushIntoTransformArray("/world", "/WorldPerceptionNeuron", &transformArray, &listener);
        pushIntoTransformArray("/WorldPerceptionNeuron", "/Hips", &transformArray, &listener);
/*        // Uncomment if vector /hipsToHand is published externally
        pushIntoTransformArray("/Hips", "/hipsToHand", &transformArray, &listener);
        transBaseLinkRightHand = transformArray.at(0).inverseTimes(transformArray.at(1).operator*=(transformArray.at(2).operator*=(
                                    transformArray.at(3))));
*/
        // Calculate transformation matrix from world to RightHand
        transBaseLinkRightHand = transformArray.at(0).inverseTimes(transformArray.at(1).operator*=(transformArray.at(2).operator*=(
                                    transHipsRightHand)));
        transformArray.clear();
        //broadcaster.sendTransform(tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(), "/base_link", "/baseLinkToHand"));

        // Rotate goal Transform M_PI around y to turn end effector into the direction of the hand/palm
        transBaseLinkRightHand = transBaseLinkRightHand.operator*(rotate);
        broadcaster.sendTransform(tf::StampedTransform(transBaseLinkRightHand, ros::Time::now(), "/base_link", "/baseLinkToHand"));




/*      // Print Goal Transform
        printf("\n Goal Transform Base Link to RightHand: \n");
        i = 0;
        printf("%1.6f %1.6f %1.6f %1.6f\n", transBaseLinkRightHand.getBasis().getColumn(i).getX(),
                    transBaseLinkRightHand.getBasis().getColumn(i).getY(),
                    transBaseLinkRightHand.getBasis().getColumn(i).getZ(),
                    transBaseLinkRightHand.getOrigin().getX());
        i = 1;
        printf("%1.6f %1.6f %1.6f %1.6f\n", transBaseLinkRightHand.getBasis().getColumn(i).getX(),
                    transBaseLinkRightHand.getBasis().getColumn(i).getY(),
                    transBaseLinkRightHand.getBasis().getColumn(i).getZ(),
                    transBaseLinkRightHand.getOrigin().getY());
        i = 2;
        printf("%1.6f %1.6f %1.6f %1.6f\n", transBaseLinkRightHand.getBasis().getColumn(i).getX(),
                    transBaseLinkRightHand.getBasis().getColumn(i).getY(),
                    transBaseLinkRightHand.getBasis().getColumn(i).getZ(),
                    transBaseLinkRightHand.getOrigin().getZ());
        printf("%1.6f %1.6f %1.6f %1.6f\n", 0.0, 0.0, 0.0, 1.0);
//*/

        // Form Goal Transform into transformation matrix T from ur_kinematics
        // TODO: COMPLETE THIS PROCESS WITH POINTERS!!!
        // is function getColumn(i) starting at 0 or 1?!
        // rotational part of transf. matrix
        for(int i = 0;i<=2;i++){
            // rotational part of transf. matrix
            transForKin[i*4+0] = transBaseLinkRightHand.getBasis().getRow(i).getX();
            transForKin[i*4+1] = transBaseLinkRightHand.getBasis().getRow(i).getY();
            transForKin[i*4+2] = transBaseLinkRightHand.getBasis().getRow(i).getZ();
        }
        // translational part
        transForKin[3] = transBaseLinkRightHand.getOrigin().getX();
        transForKin[7] = transBaseLinkRightHand.getOrigin().getY();
        transForKin[11] = transBaseLinkRightHand.getOrigin().getZ();

        transForKin[12] = 0;
        transForKin[13] = 0;
        transForKin[14] = 0;
        transForKin[15] = 1;

/**     // print Goal Transformation from base_link to RightHand
        for(int i=0;i<4;i++) {
          for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.6f ", transForKn[j]);
          printf("\n");
        }
//*/


        // Calculate Inverse Kinematic Solution
        // no solution found, if goal transform is out of reach for ur5
        // TODO: HANDLE CASE WHERE NO SOLUTION IS FOUND: robot arm should wait for user to come back
        // waitForUser();
        numSols = ur_kinematics::inverse(transForKin, qSolsArray);


        // Turn values over M_PI into negative values (by substraction of 2*M_PI) to use full range of robot, or to use limited robot
        for(int i = 0;i<numSols;i++){
            for(int j = 0;j<6;j++){
                if(M_PI < qSolsArray[i*6+j])
                    qSolsArray[i*6+j] = qSolsArray[i*6+j]-2*M_PI;
            }
        }

/**     // Print all 8 solutions to inverse kinematics / joint positions
        ROS_INFO_STREAM("number solutions: " << numSols);
        for(int i = 0;i<numSols;i++){
            printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
                qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]);
        }
        printf("\n\n");
//*/

        // TODO: !!! ONLY FOR TESTING WITHOUT ROBOT Purposes! - publish Perception Neurons initial position onto robot - to not need to fetch the robot
        // ^^^^ ^^^^ ^^^^ ^^^^ ^^^^ ^^^^
        if(counter == 0){
            jointMsg.header.stamp = ros::Time::now();
            jointMsg.position = {qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]};
            handJointPublisher.publish(jointMsg);
            // copy last published Position to lastPosition
            for (int i = 0; i < lastPosition.size(); i++)
                lastPosition.at(i) = qSolsArray[5*6+i];
        }

        // Interpolate points from rate framesPerSecPercNeuron to max 125HZ = 1 frame / 8ms
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ikSolution = {qSolsArray[ikSolNr*6+0], qSolsArray[ikSolNr*6+1], qSolsArray[ikSolNr*6+2], qSolsArray[ikSolNr*6+3], qSolsArray[ikSolNr*6+4], qSolsArray[ikSolNr*6+5]};
        calculateTrajectory(&node, &ikSolution, &lastPosition, &jointMsg, &handJointPublisher, &framesPerSecPercNeuron);


 /*       // Test: ur_kinematics package
        // inverse() and forward() find kinematics from the base link to the end effector
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//        double q[6] = {M_PI/12, M_PI/12, M_PI/12,M_PI/12 , M_PI/12, M_PI/12};
//        double* T = new double[16];
//        // M_PI/12 leads to output:
//        // -0.073 0.525 -0.848 0.312
//        // 0.980 -0.118 -0.158 0.282
//        //-0.183 -0.843 -0.506 -0.093
//        // 0.000 0.000 0.000 1.000
//        ur_kinematics::forward(q, T);
//        for(int i=0;i<4;i++) {
//          for(int j=i*4;j<(i+1)*4;j++)
//            printf("%1.3f ", T[j]);
//          printf("\n");
//        }
//        // display T in tf to check for validity
//        // tf::Transform validityTransform(tf::Matrix3x3(-0.073, 0.525, -0.848, 0.980, -0.118, -0.158,-0.183, -0.843, -0.506), tf::Vector3(0.312, 0.282, -0.093));
//        tf::Transform validityTransform(tf::Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), tf::Vector3(0.312, 0.282, -0.093));
//        // ROS_INFO_STREAM("validityTransform matrix: " << numSols);
//        broadcaster.sendTransform(tf::StampedTransform(validityTransform, ros::Time::now(), "/base_link", "/validityTransform"));

//        double qSolsArray[8*6];
//        int numSols;

//        numSols = ur_kinematics::inverse(T, qSolsArray);
//        for(int i=0;i<numSols;i++)
//        printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n",
//           qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]);
//        // useless output. Just output 5 values from zero to 2*M_PI
//        for(int i=0;i<=4;i++)
//          printf("%f ", M_PI/2.0*i);
//        printf("\n");

//        // turn values over M_PI into negative values to use full range of robot
//        // works
//        for(int i = 0;i<numSols;i++){
//            for(int j = 0;j<6;j++){
//                if(qSolsArray[i*6+j] > M_PI)
//                    qSolsArray[i*6+j] = qSolsArray[i*6+j]-2*M_PI;
//            }
//        }


        // ----------------------------
//*/

/**        // Send Data to Robot
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // initial position of ur5 axis in ... direction
        // publish angle to shoulder_pan_joint of joint_states_ur5
        sensor_msgs::JointState jointMsg;

        //Header header, string[] name, float64[] position, float64[] velocity, float64[] effort, jointMsg-header... ros::Time(NOW)

        //jointMsg.header.seq = seq;
        jointMsg.header.stamp = ros::Time::now();
        // TODO: implement correct frame_id
        jointMsg.header.frame_id = "";

        // TODO: assess names dynamically
        jointMsg.name =  jointNames;
        // jointMsg.position = {angle, angleShoulderJoint, -angleElbowJoint, 0.0, 0.0, 0.0};
        i = 5; // see above part "table for solutions" for reason
        jointMsg.position = {ikSolution.at(0), ikSolution.at(1), ikSolution.at(2), ikSolution.at(3), ikSolution.at(4), ikSolution.at(5)};
        // TODO: check if assumption is correct: every published state is going to be the next current state of ur5
        lastPosition = ikSolution;
        //jointMsg.position = {M_PI/12, M_PI/12, M_PI/12,M_PI/12 , M_PI/12, M_PI/12};
        jointMsg.velocity = {};
        jointMsg.effort = {};

        handJointPublisher.publish(jointMsg);
        // sleep(5);
        // ----------------------------
//*/
/*        // Select elbow_up solutions ((shoulder_lift_joint > 0 && elbow_joint < 0) || (shoulder_lift_joint < 0 && elbow_joint < 0))
        for(int i = 0; i < numSols; i++){
            if(qSolsArray[i*6+1] < 0 && qSolsArray[i*6+2] > 0){
                solutionIndices.push_back(i);
                printf("\n - + element: %d", i);
            }
//            else if(qSolsArray[i*6+1] > 0 && qSolsArray[i*6+2] < 0){
//                solutionIndices.push_back(i);
//                printf("\n + - element: %d", i);
//            }
//            else if(qSolsArray[i*6+1] > 0 && qSolsArray[i*6+2] > 0){
//                solutionIndices.push_back(i);
//                printf("\n + + element: %d", i); // nearly no ++ solution are foun because of hand being above the hips
//            }
            else if(qSolsArray[i*6+1] < 0 && qSolsArray[i*6+2] < 0){
                solutionIndices.push_back(i);
                printf("\n - - element: %d", i);
            }
        }
*/
/*          // test all variations of possible IK solutions
//            if(qSolsArray[i*6+1] > 0 && qSolsArray[i*6+2] < 0){
//                printf("elbow up solution + - \n");
//                jointMsg.header.stamp = ros::Time::now();
//                jointMsg.position = {qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]};
//                handJointPublisher.publish(jointMsg);
//                sleep(2);
//            }
//            if(qSolsArray[i*6+1] < 0 && qSolsArray[i*6+2] < 0){
//                printf("elbow up solution - - \n");
//                jointMsg.header.stamp = ros::Time::now();
//                jointMsg.position = {qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]};
//                handJointPublisher.publish(jointMsg);
//                sleep(2);
//            }
//            if(qSolsArray[i*6+1] < 0 && qSolsArray[i*6+2] > 0){
//                printf("elbow up solution + - \n");
//                jointMsg.header.stamp = ros::Time::now();
//                jointMsg.position = {qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]};
//                handJointPublisher.publish(jointMsg);
//                sleep(2);
//            }
//            if(qSolsArray[i*6+1] > 0 && qSolsArray[i*6+2] > 0){
//                printf("elbow up solution + + \n");
//                jointMsg.header.stamp = ros::Time::now();
//                jointMsg.position = {qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]};
//                handJointPublisher.publish(jointMsg);
//                sleep(2);
//            }
         }
*/

/*        // display all possible joint positions
        // TODO: decide which one we should consider (shoulder up/down, elbow up/down, wrist up/down)
        if(counter > 1){
//            for(int j = 0; j < solutionIndices.size(); j++){
//                i = solutionIndices.at(j);
//                jointMsg.header.stamp = ros::Time::now();
//                jointMsg.position = {qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]};
//                handJointPublisher.publish(jointMsg);
//                rate.sleep();
//            }
            // print out every solution
//            i = 0;
//            while(i < numSols){
//                jointMsg.header.stamp = ros::Time::now();
//                jointMsg.position = {qSolsArray[i*6+0], qSolsArray[i*6+1], qSolsArray[i*6+2], qSolsArray[i*6+3], qSolsArray[i*6+4], qSolsArray[i*6+5]};
//                handJointPublisher.publish(jointMsg);
//                rate.sleep();
//                i++;
//            }
        }
*/        //jointMsg.position = {qShoulderPan, qShoulderLift, qElbow, qWrist1 , qWrist2, qWrist3};

        //handJointPublisher.publish(jointMsg);
        solutionIndices.clear();
        counter++;
        //elapsedTime = ros::Time::now() - begin;
        //ROS_INFO_STREAM("elapsed time: " << elapsedTime);
        rate.sleep();
    }

    return 0;
}
