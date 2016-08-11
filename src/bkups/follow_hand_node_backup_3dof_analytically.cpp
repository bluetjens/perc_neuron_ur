#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>

// looks up Transfrom from transOrigin to transDestination in transListener and pushes them in transArray
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

  // publisher for joint_states_ur5
  // TODO: test value 10, implement correct topic
  // TODO: SET THIS VALUE TO /joint_states_ur5 AND LISTEN TO PUBLISHED DATA WITH JOINT_STATE_PUBLISHER
  // JUST CURRENTLY DOESNT WORK WITH NO APPARENT REASON
  // SO FOR NOW JUST PUBLISH JOINTSTATE DIRECTLY TO ROBOT_STATE_PUBLISHER
  ros::Publisher handJointPublisher = node.advertise<sensor_msgs::JointState>("/joint_states", 10);

  tf::TransformListener listener;
  tf::Transform transHipsRightHand; // transform from Hips to RightHand directly
  tf::Transform transShoulderRightHand; // transform from shoulder_link to RightHand directly
  tf::TransformBroadcaster broadcaster;

  // TODO: asses jointNames dynamically
  std::vector<std::string> jointNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

  // create transform array
  std::vector<tf::Transform> transformArray;

  // create string array with bodyJoints, TODO: assess all joints && prove if order is correct
  std::vector<std::string> bodyJoints{"Hips","Spine","Spine1","Spine2","Spine3","Neck","RightShoulder","RightArm","RightForeArm", "RightHand"};

  ros::Rate rate(20.0);
  while (node.ok()){
    tf::StampedTransform transform;

    // fill transformArray with transforms from "Hips" to "RightHand"
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
    // send transform with Hips as parent_node
    broadcaster.sendTransform(tf::StampedTransform(transHipsRightHand, ros::Time::now(), "Hips", "hipsToHand"));

    // get vector from Hips to RightHand
    tf::Vector3 vecHipsRightHand = transHipsRightHand.getOrigin();
    tf::Vector3 vecHipsRightHandCopy = transHipsRightHand.getOrigin();

    // project vector into xz-plane of Hips
    // projVec = (globVec ° xUnitVec) * xUnitVec + (globVec ° yUnitVec) * yUnitVec
    tf::Vector3 xUnitVec = tf::Vector3(1, 0, 0);
    tf::Vector3 zUnitVec = tf::Vector3(0, 0, 1);
    tf::Vector3 projVec = ( xUnitVec.operator*=( vecHipsRightHand.dot( xUnitVec ) ) ).operator+=(zUnitVec.operator*=( vecHipsRightHand.dot( zUnitVec ) ));
    xUnitVec = tf::Vector3 (1, 0, 0);
    zUnitVec = tf::Vector3 (0, 0, 1);
    vecHipsRightHand = vecHipsRightHandCopy;

    // DONE: test if correct vector
    // transHipsRightHand.setOrigin( projVec );
    // broadcaster.sendTransform(tf::StampedTransform(transHipsRightHand, ros::Time::now(), "Hips", "hipsToHand"));

    // find out angle / rotation between projected vector and z axis of Hips
    tfScalar angle;
    angle = projVec.angle( zUnitVec );
    // angle() only returns positive angles - convert negative ones into negative ones
    if( projVec.getX() < 0){
        angle = -angle;
    }

    //ROS_INFO_STREAM("proj Vector x: " << projVec.getX()  << " y: " << projVec.getY()  << " z: " << projVec.getZ() );
    //ROS_INFO_STREAM("unit Vector " << zUnitVec.getX()  << " y: " << zUnitVec.getY()  << " z: " << zUnitVec.getZ() );
    //ROS_INFO_STREAM("Angle between x unit vector and projected Vector in degrees: " << tfDegrees(angle));

    // calculate 2D follower analytically
    // get handHips Vec in useful coosys - shoulder_link gives perfect coosys
    // fill transformArray with transforms from "world" to "Hips" and from "world" to "shoulder_link"
    pushIntoTransformArray("WorldPerceptionNeuron", "Hips", &transformArray, &listener);
    pushIntoTransformArray("world", "WorldPerceptionNeuron", &transformArray, &listener);
    pushIntoTransformArray("world", "base_link", &transformArray, &listener);
    pushIntoTransformArray("base_link", "shoulder_link", &transformArray, &listener);

    // calculate transformation matrix from shoulder_link to RightHand
    transShoulderRightHand = transformArray.at(3).inverseTimes(transformArray.at(2).inverseTimes(
                                transformArray.at(1).operator*=(transformArray.at(0).operator*=(
                                transHipsRightHand))));
    transformArray.clear();
    broadcaster.sendTransform(tf::StampedTransform(transShoulderRightHand, ros::Time::now(), "shoulder_link", "shoulderToHand"));

    // get parameters for inverse kinematics calculation of ur5 2DOF arm
    tf::Vector3 vecShoulderRightHand = transShoulderRightHand.getOrigin();
    tfScalar xDis = vecShoulderRightHand.getX();
    tfScalar zDis = vecShoulderRightHand.getZ();
    ROS_INFO_STREAM("Vector shoulder right hand x: " << xDis  << " z: " << zDis << " angle to X: " << vecShoulderRightHand.angle( tf::Vector3(1, 0, 0) ) );

    pushIntoTransformArray("upper_arm_link", "forearm_link", &transformArray, &listener);
    pushIntoTransformArray("forearm_link", "wrist_1_link", &transformArray, &listener);
    tfScalar l1 = transformArray.at(0).getOrigin().getZ();
    tfScalar l2 = transformArray.at(1).getOrigin().getZ();
    transformArray.clear();

    tfScalar angleElbowJoint;
    ROS_INFO_STREAM("robot " << " l1: " << l1 << " l1: " << l2);
    // firstAtan2 = [xDis² + zDis² - l1² - l2²] / [2 * l1 * l2]
    tfScalar firstAtan2 =  ( tfPow(xDis, 2) + tfPow(zDis, 2) - tfPow(l1, 2) - tfPow(l2, 2) ) / ( 2 * l1 * l2 );
    // firstAtan1 = +- sqrt ( 1 - ( firstAtan2 )²)
    tfScalar firstAtan1 = tfSqrt( 1 - tfPow(firstAtan2, 2) );
    angleElbowJoint = tfAtan2( firstAtan1, firstAtan2 );

    tfScalar angleShoulderJoint;
    // secondAtan2 = [xDis * (l1 + l2*cos(angle2)) + zDis * l2 * sin(angle2)] / [xDis² + zdis²]
    tfScalar secondAtan2 = ( xDis * ( l1 + l2 * tfCos( angleElbowJoint ))
                                + zDis * l2 * tfSin( angleElbowJoint ))
                                / ( tfPow(xDis,2) + tfPow(zDis, 2) );
    // secondAtan1 = +- sqrt ( 1 - ( firstAtan2 )²)
    tfScalar secondAtan1 = tfSqrt( 1 - tfPow(secondAtan2, 2) );
    angleShoulderJoint = tfAtan2( secondAtan1, secondAtan2 );

    ROS_INFO_STREAM("angleElbowJoint: " << tfDegrees(angleElbowJoint) << " angleShoulderJoint: " << tfDegrees(angleShoulderJoint) << " l1: " << l1);

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
    joint_msg.position = {angle, angleShoulderJoint, -angleElbowJoint, 0.0, 0.0, 0.0};
    //joint_msg.position = {M_PI/12, M_PI/12, M_PI/12,M_PI/12 , M_PI/12, M_PI/12};
    joint_msg.velocity = {};
    joint_msg.effort = {};

    handJointPublisher.publish(joint_msg);

// failed transformations:

//    tf::Quaternion quaternion;
//    tf::Vector3 origin;
//    tfScalar angle;
//    tfScalar angle2;
//    tfScalar angle3;

//    quaternion = transform.getRotation();
//    origin = transform.getOrigin();
//    angle = quaternion.getAngle();
//    angle2 = transform2.getRotation().getAngle();
//    angle3 = transform3.getRotation().getAngle();

//    // DONE : test if incoming data equals sent data by perc_neuron_tf_broadcaster
//    ROS_INFO_STREAM("Incoming transform: getOrigin: y: " << origin.y() << " x: " << origin.x() << " z: " << origin.z());
//    ROS_INFO_STREAM("Incoming transform: getRotation: qy: " << quaternion.getAxis().y() << " qx: " << quaternion.getAxis().x() << " qz: " << quaternion.getAxis().z() << " qw: " << quaternion.getW());
//    ROS_INFO_STREAM("Incoming transform: Quaternion Angle in rad: " << quaternion.getAngle());

//    sensor_msgs::JointState joint_msg;

//    //Header header, string[] name, float64[] position, float64[] velocity, float64[] effort, joint_msg-header... ros::Time(NOW)

//    //joint_msg.header.seq = seq;
//    joint_msg.header.stamp = ros::Time::now();
//    // TODO: implement correct frame_id
//    joint_msg.header.frame_id = "";

//    // TODO: assess names dynamically
//    joint_msg.name =  jointNames;
//    joint_msg.position = {0.0, angle, angle2, angle3, 0.0, 0.0};
//    joint_msg.velocity = {};
//    joint_msg.effort = {};

//    handJointPublisher.publish(joint_msg);

    rate.sleep();
  }
  return 0;
}
