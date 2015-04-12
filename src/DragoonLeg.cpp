#include "DragoonLeg.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ik.h"

DragoonLeg::DragoonLeg(int id, double ox, double oy, double oz)
    : id(id)
    , origX(ox)
    , origY(oy)
    , origZ(oz)
    , defaultDuration(1.0)
{
    ros::NodeHandle nh;
    std::stringstream name;
    name.str("");
    name << "/leg_controller_" << (id+1) << "/command";
    _legPublisher = nh.advertise<trajectory_msgs::JointTrajectory > (name.str(), 1);
    ROS_INFO("Publish : %s", _legPublisher.getTopic().c_str());
}

int DragoonLeg::setOrigin(double x, double y, double z)
{
    origX = x;
    origY = y;
    origZ = z;

    return 0;
}

int DragoonLeg::setOriginZ(double z)
{
    origZ = z;
    return 0;
}

int DragoonLeg::moveRelative(double x, double y, double z, double duration)
{
    return moveAbsolute(origX + x, origY + y, origZ + z, duration);
}

int DragoonLeg::moveAbsolute(double x, double y, double z, double duration)
{
    double theta1, theta2, theta3;
    ROS_DEBUG("Move leg %d to %f, %f, %f", id+1, x, y, z);
    trajectory_msgs::JointTrajectory legCommand;
    trajectory_msgs::JointTrajectoryPoint legPosition;
    legCommand.joint_names.resize(3);

    std::stringstream jointName;
    for( int j = 0; j < 3; j++ ) {
        jointName.str("");
        jointName << "joint_" << (id+1) << "_" << (j+1);
        legCommand.joint_names[j] = jointName.str();
        //ROS_INFO("JOINT : %s", jointName.str().c_str());
    } 


    if ( getTheta( x, y, z, theta1, theta2, theta3) == 0 ) {
        ROS_DEBUG("t1 = %f, t2 = %f, t3 = %f", theta1, theta2, theta3);
        legCommand.header.stamp = ros::Time::now();
        legCommand.header.frame_id = "leg_base";

        legCommand.points.resize(1);
        legPosition.positions.resize(3);
        legPosition.positions[0] = theta1;
        legPosition.positions[1] = theta2;
        legPosition.positions[2] = theta3;
        legCommand.points[0] = legPosition;
        legCommand.points[0].time_from_start = ros::Duration(duration);

        ROS_DEBUG("Publish to %s", _legPublisher.getTopic().c_str());
        _legPublisher.publish(legCommand);
        return 0;
    }

    return -1;

}


