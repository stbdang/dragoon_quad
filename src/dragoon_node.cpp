// TODO
// 1. Implement stop - one empty cycle of moving legs back to origin
// 2. Implement subscriber mechanism - listen to direction/speed pair.


#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Twist.h"
#include "DragoonLeg.h"
#include "CreepGait.h"

#define PI 3.14159265

enum MoveState
{
    MS_STOPPED,
    MS_STOPPING,
    MS_MOVING
} typedef MoveState;

static MoveState requestedState = MS_STOPPED;
static double reqStepSize = 0;
static double reqDirection = 0;

double convertRadToDeg(double rad)
{
    return rad * 180 / PI;
}

void cmdCallback( const ros::MessageEvent<geometry_msgs::Twist const>& event )
{
    const geometry_msgs::Twist& msg = *event.getMessage();

    ROS_INFO("Twist msg : [%f %f %f] [%f %f %f]", 
            msg.linear.x, 
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z);
    
    double x = msg.linear.x;
    double y = msg.linear.y;

    double angle = convertRadToDeg( atan2(x, y) );
    double length = sqrt(x*x + y*y);
    length = length * 1.5;

    ROS_INFO("Length : %lf Angle : %lf", length, angle);

    if ( length < 1.0 ) {
        reqStepSize = 0;
        requestedState = MS_STOPPED;
    } else {
        requestedState = MS_MOVING;
        reqStepSize = length;
    }

    if ( angle < 0.0 ) {
        angle += 360.0;
    }

    reqDirection = angle;

}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "dragoon_node" );
    ros::NodeHandle nh;

    DragoonLeg *legs[4];
    
    // Instantiate legs
    ROS_INFO("Advertising topics");
    for (int i = 0; i < 4; i++) {
        legs[i] = new DragoonLeg(i, 0, 0, 0);
    }
    
    // Setup subscribe
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>( "/dragoon_cmd", 1, cmdCallback ); 
    ros::Rate rate(4);
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
        legs[i]->setOriginZ(45);
        legs[i]->moveRelative(0, 0, 0, 2);
    }
    ros::spinOnce();
    rate.sleep();
    sleep(2);

    
    int count = 0;
    int walk = 0;
    double dur = 0.15;
    double step_size = 0;
    double direction = 0;
    Gait *gait = new CreepGait();
    MoveState state = MS_STOPPED;
    while (nh.ok() ) {

        if ( state == MS_MOVING ) {
            LegOffset offset[4];
            memset(offset, 0, sizeof(offset));

            ROS_INFO("Moving : %f %f", step_size, direction);
            gait->setDirection(direction);
            gait->setStepSize(step_size);

            if ( gait->generateMove(count, offset) == 0 ) {
                for (int i=0; i < 4; i++ ) {
                    legs[i]->moveRelative(offset[i].x,
                            offset[i].y, offset[i].z, dur);
                }
            }

            count++;
            if ( count >= gait->get_num_phase() ) {
                count = count % gait->get_num_phase();

                if ( requestedState == MS_STOPPED ) {
                    state = MS_STOPPED; 
                } else {
                    state = requestedState;
                }
                step_size = reqStepSize;
                direction = reqDirection;
            }

        } else if ( state == MS_STOPPED ) {
            ROS_INFO("Stopped");
            state = requestedState;
            step_size = reqStepSize;
            direction = reqDirection;
        }

        ros::spinOnce();
        rate.sleep();
    }
    
    delete gait; 
    return 0;
}

