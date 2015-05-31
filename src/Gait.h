#ifndef GAIT_H
#define GAIT_H

#define NUM_LEG 4
struct LegOffset
{
    double x;
    double y;
    double z;
};

class Gait 
{
public:
    virtual ~Gait() {};

    void setDirection(double direction);
    void setStepSize(double step_size);

    virtual int generateMove(int phase, struct LegOffset offset[]) = 0; 
    int get_num_phase();

protected:
    Gait(int num_phase) : _num_phase(num_phase) {}
    const int _num_phase;
    double _direction;
    double _step_size;

private:
};

#endif // GAIT_H
