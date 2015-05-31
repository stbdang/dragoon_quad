#include "Gait.h"

void Gait::setDirection(double direction) { _direction = direction; }
void Gait::setStepSize(double step_size) { _step_size = step_size; }

int Gait::get_num_phase() { return _num_phase; };
