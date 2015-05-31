#ifndef CREEPGAIT_H
#define CREEPGAIT_H

#define NUM_PHASE 10

#include "Gait.h"

class CreepGait : public Gait
{
public:
    CreepGait();
    ~CreepGait();
    int generateMove(int phase, struct LegOffset offset[]); 

private:
    int step_layer_0(int phase, struct LegOffset offset[]);
    void step_layer_1(int phase, struct LegOffset offset[]);
    int _lean[NUM_PHASE];
    int _legorder[NUM_LEG];
};

#endif // CREEPGAIT_H
