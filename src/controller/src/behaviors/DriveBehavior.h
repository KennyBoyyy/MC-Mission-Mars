#ifndef DRIVE_BEHAVIOR_H
#define DRIVE_BEHAVIOR_H


#include "BehaviorInterface.h"

#include "../controllers/DriveController.h"

class DriveBehavior : public Behavior{
    float x=0, y=0;

    public:
        DriveBehavior(double x, double y) : Behavior(DRIVE_BEHAVIOR_TYPE, true){
            this->x = x;
            this->y = y;
        }
        bool tick();
};







#endif
