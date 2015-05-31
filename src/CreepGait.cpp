#include "CreepGait.h"
#include <cmath>

#define PI 3.14159265

CreepGait::CreepGait()
    : Gait(NUM_PHASE)
{
    _lean[0]= -1; 
    _lean[1]= -1;
    _lean[2]= 1;
    _lean[3]= 1;
    _lean[4]= 1;
    _lean[5]= 1;
    _lean[6]= 1;
    _lean[7]= -1;
    _lean[8]= -1;
    _lean[9]= -1;

}

CreepGait::~CreepGait()
{
}

int CreepGait::generateMove(int phase, struct LegOffset offset[])
{
    if ( step_layer_0(phase, offset) < 0 ) {
        return -1;
    }
    step_layer_1(phase, offset);

    return 0;
}

int CreepGait::step_layer_0(int phase, struct LegOffset offset[])
{
    double delta = 20;
    double high = -40;

    int legside[4];
    // 0 : 3 2 4 1
    // 90 : 4 1 2 3
    // 180 : 2 3 1 4
    // 270 : 1 4 3 2

    if ( _direction < 0 || _direction >= 360 ) {
        //ROS_ERROR("Invalid direction");
        return -1;
    }

    if ( _direction > 315 || _direction < 45 ) {
        _legorder[0] = 8;
        _legorder[1] = 3;
        _legorder[2] = 0;
        _legorder[3] = 5;
    } else if (_direction < 135) {
        _legorder[0] = 3;
        _legorder[1] = 5;
        _legorder[2] = 8;
        _legorder[3] = 0;
    } else if (_direction < 225) {
        _legorder[0] = 5;
        _legorder[1] = 0;
        _legorder[2] = 3;
        _legorder[3] = 8;
    } else {
        _legorder[0] = 0;
        _legorder[1] = 8;
        _legorder[2] = 5;
        _legorder[3] = 3;
    }

    // lateral movement perpendicular to the direction of walk
    double rightAngle = ( _direction + 90.0 );
    rightAngle = (rightAngle > 360.0) ? rightAngle - 360.0 : rightAngle;
    double dX = delta * sin( rightAngle * PI/180.0);
    double dY = delta * cos( rightAngle * PI/180.0);
    int legToLift = -1;

    if ( phase == _legorder[0] ) {
        // Leg 1
        legToLift = 0;
    } else if ( phase == _legorder[1] ) {
        // Leg 2
        legToLift = 1;
    } else if ( phase == _legorder[2] ) {
        // Leg 3
        legToLift = 2;
    } else if ( phase == _legorder[3] ) {
        // Leg 4
        legToLift = 3;
    }

    for (int i = 0; i < 4; i++ ) {
        offset[i].x = _lean[phase]*dX;
        offset[i].y = _lean[phase]*dY;
        offset[i].z = 0;
    }

    if ( legToLift >= 0 ) {
        offset[legToLift].z = high;
    }

    return 0;
}

void CreepGait::step_layer_1(int phase, struct LegOffset offset[])
{
    // leg order : 3, 2, 4, 1

    for ( int leg = 0; leg < NUM_LEG; leg++ ) {
        int lphase = (phase + _num_phase - _legorder[leg]) % _num_phase;

        double xmov = _step_size * sin(_direction * PI/180);
        double ymov = _step_size * cos(_direction * PI/180);
        if ( lphase < 2 ) {
            offset[leg].x += (xmov / 2) * (lphase+1);
            offset[leg].y += (ymov / 2) * (lphase+1);
        } else {
            lphase = lphase - 2;
            offset[leg].x += (-xmov / (_num_phase - 2)) * (lphase+1);
            offset[leg].y += (-ymov / (_num_phase - 2)) * (lphase+1);
        }
    }
}

