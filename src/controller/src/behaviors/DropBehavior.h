#ifndef DROP_BEHAVIOR_H
#define DROP_BEHAVIOR_H

#include "BehaviorInterface.h"
#include "../handlers/Handlers.h"


class DropBehavior : public Behavior{
    enum Stages{
        INIT = 0,
        DRIVE_TO_CENTER,
        DROP_CUBE,
        DRIVE_BACK
    };

    Stages stage = DRIVE_TO_CENTER;

    //The initial x and y from whch we drive
    double x = 0;
    double y = 0;

    int slowDrive = 60;

    public:
        DropBehavior() : Behavior(DROP_BEHAVIOR_TYPE){}
        bool tick();
};

#endif
