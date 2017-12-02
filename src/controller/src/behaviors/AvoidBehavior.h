#ifndef AVOID_BEHAVIOR_H
#define AVOID_BEHAVIOR_H

#include "Behaviors.h"
#include <time.h>

class AvoidBehavior : public Behavior{
    int waitTime;
    time_t initTime;

public:
    AvoidBehavior();
    bool tick();

};









#endif
