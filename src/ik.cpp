#include "ik.h"
#include <cmath>
#include <ros/ros.h>

#define L0 27
#define L1 75
#define L2 89

#define PI 3.14159265

double convertRadToDeg(double rad)
{
    return rad * 180 / PI;
}

int getTheta(double x, double y, double z, double& theta1, double& theta2, double& theta3)
{
    if ( z < 0 || z > 100 ) {
        // We don't want leg to be up yet.
        // Range is too high
        ROS_ERROR("Z bad range");
        return -1;
    }

    double r = sqrt( (x*x) + (y*y) + (z*z) );
    if ( r < 65 || r > 150 ) {
        // Range is too high
        ROS_ERROR("X,Y bad range : %f", r);
        return -1;
    }

    double L = sqrt( z*z + pow(r-L0, 2) );
    double a1 = acos(z/L);
    double a2 = acos((L2*L2 - L1*L1 - L*L)/(-2 * L1 * L));

    theta1 = convertRadToDeg( atan2 (y, x) );
    if ( theta1 < 0 )
        theta1 += 360;

    theta2 = convertRadToDeg( PI - (a1 + a2) );
    theta3 = convertRadToDeg( PI - (acos( (L*L - L2*L2 - L1*L1) / (-2 * L1 * L2))));
    
    return 0;
}
