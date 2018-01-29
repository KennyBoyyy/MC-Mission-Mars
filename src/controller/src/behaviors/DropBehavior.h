#ifndef DROP_BEHAVIOR_H
#define DROP_BEHAVIOR_H

#include "BehaviorInterface.h"

#include "../controllers/DriveController.h"

class DropBehavior : public Behavior{
    enum Stages{
        ODOM_TARGET = 0,
        GPS_TARGET,
        SEARCH,
        DROP
    };

    public:
        DropBehavior() : Behavior(DROP_BEHAVIOR_TYPE){

        }
};














#endif
