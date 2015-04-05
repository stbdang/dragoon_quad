
#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "DragoonLeg.h"

struct LegOffset
{
    double x;
    double y;
    double z;
};

static struct LegOffset offset[4];

void step_layer_0(int count)
{
    double delta = 20;
    double high = -40;
    switch (count) {
        // Leg 1
        case 6:
            for (int i = 0; i < 4; i++ ) {
                offset[i].x = -delta;
                offset[i].y = -delta;
                offset[i].z = 0;
            }
            offset[0].z = high;
            break;
        // Leg 2
        case 2:
            for (int i = 0; i < 4; i++ ) {
                offset[i].x = delta;
                offset[i].y = -delta;
                offset[i].z = 0;
            }
            offset[1].z = high;
            break;
        // Leg 3
        case 0:
            for (int i = 0; i < 4; i++ ) {
                offset[i].x = -delta;
                offset[i].y = delta;
                offset[i].z = 0;
            }
            offset[2].z = high;
            break;
        // Leg 4
        case 4:
            for (int i = 0; i < 4; i++ ) {
                offset[i].x = delta;
                offset[i].y = delta;
                offset[i].z = 0;
            }
            offset[3].z = high;
            break;
    }

}

void step_layer_1(int leg, int count)
{
    // leg order : 3, 2, 4, 1

    switch (leg) {
        case 1:
            count = ( count + 6 ) % 8;
            break;
        case 3:
            count = ( count + 4 ) % 8;
            break;
        case 0:
            count = ( count + 2 ) % 8;
            break;
    }

    double step_size = 20;
    if ( count < 2 ) {
        offset[leg].y += (step_size / 2) * (count+1);
    } else {
        count = count - 2;
        offset[leg].y += (-step_size / 6) * (count+1);
    }

}

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
    double dur = 0.5;
    while (nh.ok() ) {
        memset(offset, 0, sizeof(offset));
        
        step_layer_0(count);

        for (int i=0; i < 4; i++ ) {
            step_layer_1(i, count);
            legs[i]->moveRelative(offset[i].x, offset[i].y, offset[i].z, dur);
        }


        ros::spinOnce();
        rate.sleep();
        count = (count + 1 ) %8;
    }
    
    
    return 0;
}
