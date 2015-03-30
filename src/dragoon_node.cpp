
#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "DragoonLeg.h"

int main( int argc, char** argv )
{
    ros::init( argc, argv, "dragoon_node" );
    ros::NodeHandle nh;

    DragoonLeg *legs[4];
    
    // Setup legs
    
    ROS_INFO("Advertising topics");
    for (int i = 0; i < 4; i++) {
        legs[i] = new DragoonLeg(i, 0, 0, 0);
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

        legs[i]->setOrigin(legPosX, legPosY, legPosZ);
        
        if ( legs[i]->moveRelative(0, 0, 0, 2) < 0 ) {
            ROS_ERROR("Move leg %d failed", i);
        }
    }

    sleep(2);
    ros::spinOnce();
    rate.sleep();
    // Stand up
    for (int i = 0; i < 4; i++) {
        legs[i]->setOriginZ(70);
        legs[i]->moveRelative(0, 0, 0, 3);
    }
    ros::spinOnce();
    rate.sleep();
    sleep(2);

    
    int count = 0;
    double delta = 15;
    double high = -40;
    double dur = 0.5;
    while (nh.ok() ) {
        switch (count) {
            case 0:
                legs[0]->moveRelative(-delta, -delta, high, dur);
                legs[1]->moveRelative(-delta, -delta, 0,    dur);
                legs[2]->moveRelative(-delta, -delta, 0,    dur);
                legs[3]->moveRelative(-delta, -delta, 0,    dur);
                break;
            case 1:
                legs[0]->moveRelative(0,    0,      0,      dur);
                legs[1]->moveRelative(0,    0,      0,      dur);
                legs[2]->moveRelative(0,    0,      0,      dur);
                legs[3]->moveRelative(0,    0,      0,      dur);
                break;
            case 2:
                legs[0]->moveRelative(delta, -delta, 0,     dur);
                legs[1]->moveRelative(delta, -delta, high,  dur);
                legs[2]->moveRelative(delta, -delta, 0,     dur);
                legs[3]->moveRelative(delta, -delta, 0,     dur);
                break;
            case 3:
                legs[0]->moveRelative(0,    0,      0,      dur);
                legs[1]->moveRelative(0,    0,      0,      dur);
                legs[2]->moveRelative(0,    0,      0,      dur);
                legs[3]->moveRelative(0,    0,      0,      dur);
                break;
            case 4:
                legs[0]->moveRelative(-delta,  delta, 0,    dur);
                legs[1]->moveRelative(-delta,  delta, 0,    dur);
                legs[2]->moveRelative(-delta,  delta, high, dur);
                legs[3]->moveRelative(-delta,  delta, 0,    dur);
                break;
            case 5:
                legs[0]->moveRelative(0,    0,      0,      dur);
                legs[1]->moveRelative(0,    0,      0,      dur);
                legs[2]->moveRelative(0,    0,      0,      dur);
                legs[3]->moveRelative(0,    0,      0,      dur);
                break;
            case 6:
                legs[0]->moveRelative( delta,  delta, 0,    dur);
                legs[1]->moveRelative( delta,  delta, 0,    dur);
                legs[2]->moveRelative( delta,  delta, 0,    dur);
                legs[3]->moveRelative( delta,  delta, high, dur);
                break;
            case 7:
                legs[0]->moveRelative(0,    0,      0,      dur);
                legs[1]->moveRelative(0,    0,      0,      dur);
                legs[2]->moveRelative(0,    0,      0,      dur);
                legs[3]->moveRelative(0,    0,      0,      dur);
                break;
        }

        ros::spinOnce();
        rate.sleep();
        count = (count + 1 ) %8;
    }
    
    
    return 0;
}
