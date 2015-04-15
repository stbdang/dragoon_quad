// TODO
// 1. Implement stop - one empty cycle of moving legs back to origin
// 2. Implement subscriber mechanism - listen to direction/speed pair.


#include <ros/ros.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "DragoonLeg.h"

#define PI 3.14159265

struct LegOffset
{
    double x;
    double y;
    double z;
};

enum MoveState
{
    MS_STOPPED,
    MS_STOPPING,
    MS_MOVING
} typedef MoveState;

static struct LegOffset offset[4];
static int legorder[4];
static int numPhase = 10;
static MoveState requestedState = MS_STOPPED;
static double reqStepSize = 0;

void step_layer_0(int count, double direction)
{
    double delta = 20;
    double high = -40;

    int legside[4];
    int lean[numPhase];
    // 0 : 3 2 4 1
    // 90 : 4 1 2 3
    // 180 : 2 3 1 4
    // 270 : 1 4 3 2

    if ( direction < 0 || direction >= 360 ) {
        ROS_ERROR("Invalid direction");
        return;
    }

    lean[0]= -1; 
    lean[1]= -1;
    lean[2]= 1;
    lean[3]= 1;
    lean[4]= 1;
    lean[5]= 1;
    lean[6]= 1;
    lean[7]= -1;
    lean[8]= -1;
    lean[9]= -1;

    if ( direction > 315 || direction < 45 ) {
        legorder[0] = 8;
        legorder[1] = 3;
        legorder[2] = 0;
        legorder[3] = 5;
        
    } else if (direction < 135) {
        legorder[0] = 3;
        legorder[1] = 5;
        legorder[2] = 8;
        legorder[3] = 0;
    } else if (direction < 225) {
        legorder[0] = 5;
        legorder[1] = 0;
        legorder[2] = 3;
        legorder[3] = 8;
    } else {
        legorder[0] = 0;
        legorder[1] = 8;
        legorder[2] = 5;
        legorder[3] = 3;
    }
#if 0
// Simple opposite to lifting leg
    if ( count == legorder[0]*2 ) {
        //LEG 1
        for (int i = 0; i < 4; i++ ) {
            offset[i].x = -delta;
            offset[i].y = -delta;
            offset[i].z = 0;
        }
        offset[0].z = high;
    } else if ( count == legorder[1]*2 ) {
        // Leg 2
        for (int i = 0; i < 4; i++ ) {
            offset[i].x = delta;
            offset[i].y = -delta;
            offset[i].z = 0;
        }
        offset[1].z = high;
    } else if ( count == legorder[2]*2 ) {
        // Leg 3
        for (int i = 0; i < 4; i++ ) {
            offset[i].x = -delta;
            offset[i].y = delta;
            offset[i].z = 0;
        }
        offset[2].z = high;
    } else if ( count == legorder[3]*2 ) {
        // Leg 4
        for (int i = 0; i < 4; i++ ) {
            offset[i].x = delta;
            offset[i].y = delta;
            offset[i].z = 0;
        }
        offset[3].z = high;
    }
#endif

    // lateral movement perpendicular to the direction of walk
    double rightAngle = ( direction + 90.0 );
    rightAngle = (rightAngle > 360.0) ? rightAngle - 360.0 : rightAngle;
    double dX = delta * sin( rightAngle * PI/180.0);
    double dY = delta * cos( rightAngle * PI/180.0);
    int legToLift = -1;

    if ( count == legorder[0] ) {
        //LEG 1
        legToLift = 0;
    } else if ( count == legorder[1] ) {
        // Leg 2
        legToLift = 1;
    } else if ( count == legorder[2] ) {
        // Leg 3
        legToLift = 2;
    } else if ( count == legorder[3] ) {
        // Leg 4
        legToLift = 3;
    }

    for (int i = 0; i < 4; i++ ) {
        offset[i].x = lean[count]*dX;
        offset[i].y = lean[count]*dY;
        offset[i].z = 0;
    }

    if ( legToLift >= 0 ) {
            offset[legToLift].z = high;
    }

}

void step_layer_1(int leg, int count, double direction, double step_size)
{
    // leg order : 3, 2, 4, 1
    count = (count + numPhase - legorder[leg]) % numPhase;

    double xmov = step_size * sin(direction * PI/180);
    double ymov = step_size * cos(direction * PI/180);
    if ( count < 2 ) {
        offset[leg].x += (xmov / 2) * (count+1);
        offset[leg].y += (ymov / 2) * (count+1);
    } else {
        count = count - 2;
        offset[leg].x += (-xmov / (numPhase - 2)) * (count+1);
        offset[leg].y += (-ymov / (numPhase - 2)) * (count+1);
    }

}

void cmdCallback( const ros::MessageEvent<

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
    double dur = 0.25;
    double direction = 0.0;
    MoveState state = MS_STOPPED;
    while (nh.ok() ) {
        memset(offset, 0, sizeof(offset));

        if ( state == MS_MOVING ) {
            step_layer_0(count, direction);

            for (int i=0; i < 4; i++ ) {
                step_layer_1(i, count, direction);
                legs[i]->moveRelative(offset[i].x, offset[i].y, offset[i].z, dur, reqStepSize);
            }

            count++;
            if ( count >= numPhase ) {
                count = count % numPhase;
                //walk++;

                if ( requestedState == MS_STOPPED ) {
                    state = MS_STOPPING 
                } else {
                    state = requestedState;
                }
            }

            //if ( walk >= 5 ) {
            //    walk = 0;
            //    direction = direction + 90.0;
            //    if ( direction >= 360.0 ) {
            //        direction = 0;
            //    }
            //    ROS_INFO("Changing direction to %f", direction);
            //    sleep(1);
            //}
        } else if ( state == MS_STOPPING ) {
            step_layer_0(count, direction);

            for (int i=0; i < 4; i++ ) {
                step_layer_1(i, count, direction);
                legs[i]->moveRelative(offset[i].x, offset[i].y, offset[i].z, dur, 0);
            }

            count++;
            if ( count >= numPhase ) {
                count = count % numPhase;
                state = requestedState;
            }

        } else if ( state == MS_STOPPED ) {
            state = requestedState;
        }

        ros::spinOnce();
        rate.sleep();
    }
    
    
    return 0;
}
