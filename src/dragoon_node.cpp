
#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "ik.h"

static ros::Publisher legs[4]; 

int moveLeg( int leg, double x, double y, double z )
{
    ROS_INFO("Move leg %d to %f, %f, %f", leg+1, x, y, z);
    trajectory_msgs::JointTrajectory legCommand;
    trajectory_msgs::JointTrajectoryPoint legPosition;
    legCommand.joint_names.resize(3);
    
    std::stringstream jointName;
    for( int j = 0; j < 3; j++ ) {
        jointName.str("");
        jointName << "joint_" << (leg+1) << "_" << (j+1);
        legCommand.joint_names[j] = jointName.str();
        ROS_INFO("JOINT : %s", jointName.str().c_str());
    } 

    double theta1, theta2, theta3;
    if ( getTheta(x, y, z, theta1, theta2, theta3) == 0 ) {
        ROS_INFO("t1 = %f, t2 = %f, t3 = %f", theta1, theta2, theta3);
        legCommand.header.stamp = ros::Time::now();
        legCommand.header.frame_id = "leg_base";

        legCommand.points.resize(1);
        legPosition.positions.resize(3);
        legPosition.positions[0] = theta1;
        legPosition.positions[1] = theta2;
        legPosition.positions[2] = theta3;
        legCommand.points[0] = legPosition;
        legCommand.points[0].time_from_start = ros::Duration(0.5);

        ROS_INFO("Publish to %s", legs[leg].getTopic().c_str());
        legs[leg].publish(legCommand);
        return 0;
    }

    return -1;
    
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "dragoon_node" );
    ros::NodeHandle nh;
    
    // Setup topic publisher
    
    ROS_INFO("Advertising topics");
    for (int i = 0; i < 4; i++) {
        std::stringstream name;
        name.str("");
        name << "/leg_controller_" << (i+1) << "/command";
        legs[i] = nh.advertise<trajectory_msgs::JointTrajectory > (name.str(), 1);
        ROS_INFO("Publish : %s", legs[i].getTopic().c_str());
    }
    
    ros::Rate rate(2);

    ROS_INFO("Initialize legs");
    ros::spinOnce();
    rate.sleep();

    // Initialize legs      
    double initX = 70, initY = 70;
    for (int i = 0; i < 4; i++) {
        double legPosX, legPosY;
        double legPosZ = 5;
        if ( i == 0 ) {
            legPosX = -1 * initX;
            legPosY = -1 * initY;
        } else if ( i == 1 ) {
            legPosX = initX;
            legPosY = -1 * initY;
        } else if ( i == 2 ) {
            legPosX = -1 * initX;
            legPosY = initY;
        } else if ( i == 3 ) {
            legPosX = initX;
            legPosY = initY;
        } 
        
        if ( moveLeg(i, legPosX, legPosY, legPosZ) < 0 ) {
            ROS_ERROR("Move leg %d failed", i);
        }
    }

    // Stand up
    for ( int meh = 1; meh <= 4; meh++ ) {

        for (int i = 0; i < 4; i++) {
            double legPosX, legPosY;
            double legPosZ = 5 + meh * 15;
            if ( i == 0 ) {
                legPosX = -1 * initX;
                legPosY = -1 * initY;
            } else if ( i == 1 ) {
                legPosX = initX;
                legPosY = -1 * initY;
            } else if ( i == 2 ) {
                legPosX = -1 * initX;
                legPosY = initY;
            } else if ( i == 3 ) {
                legPosX = initX;
                legPosY = initY;
            } 
            
            if ( moveLeg(i, legPosX, legPosY, legPosZ) < 0 ) {
                ROS_ERROR("Move leg %d failed", i);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    
    int count = 0;
    double delta = 15;
    double low = 70;
    double high = 30;
    while (nh.ok() ) {
        switch (count) {
            case 0:
                moveLeg(0, -70 - delta, -70 -delta, high);
                moveLeg(1,  70 - delta, -70 -delta, low);
                moveLeg(2, -70 - delta,  70 -delta, low);
                moveLeg(3,  70 - delta,  70 -delta, low);
                break;
            case 1:
                moveLeg(0, -70, -70, low);
                moveLeg(1,  70, -70, low);
                moveLeg(2, -70,  70, low);
                moveLeg(3,  70,  70, low);
                break;
            case 2:
                moveLeg(0, -70 + delta, -70 - delta, low);
                moveLeg(1,  70 + delta, -70 - delta, high);
                moveLeg(2, -70 + delta,  70 - delta, low);
                moveLeg(3,  70 + delta,  70 - delta, low);
                break;
            case 3:
                moveLeg(0, -70, -70, low);
                moveLeg(1,  70, -70, low);
                moveLeg(2, -70,  70, low);
                moveLeg(3,  70,  70, low);
                break;
            case 4:
                moveLeg(0, -70 - delta, -70 + delta, low);
                moveLeg(1,  70 - delta, -70 + delta, low);
                moveLeg(2, -70 - delta,  70 + delta, high);
                moveLeg(3,  70 - delta,  70 + delta, low);
                break;
            case 5:
                moveLeg(0, -70, -70, low);
                moveLeg(1,  70, -70, low);
                moveLeg(2, -70,  70, low);
                moveLeg(3,  70,  70, low);
                break;
            case 6:
                moveLeg(0, -70 + delta, -70 + delta, low);
                moveLeg(1,  70 + delta, -70 + delta, low);
                moveLeg(2, -70 + delta,  70 + delta, low);
                moveLeg(3,  70 + delta,  70 + delta, high);
                break;
            case 7:
                moveLeg(0, -70, -70, low);
                moveLeg(1,  70, -70, low);
                moveLeg(2, -70,  70, low);
                moveLeg(3,  70,  70, low);
                break;
        }

        ros::spinOnce();
        rate.sleep();
        count = (count + 1 ) %8;
    }
    
    
    return 0;
}
