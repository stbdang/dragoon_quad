#ifndef DRAGOONLEG_H
#define DRAGOONLEG_H

#include <ros/ros.h>

class DragoonLeg
{
public:
    DragoonLeg( int id, double ox, double oy, double oz);
    int setOrigin(double x, double y, double z);
    int setOriginZ(double z);
    int moveRelative(double x, double y, double z, double duration);
    int moveAbsolute(double x, double y, double z, double duration);

private:
    int id;
    double origX;
    double origY;
    double origZ;
    double defaultDuration;

    ros::Publisher _legPublisher;

};

#endif
