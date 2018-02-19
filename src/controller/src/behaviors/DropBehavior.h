#ifndef DROP_BEHAVIOR_H
#define DROP_BEHAVIOR_H

#include "BehaviorInterface.h"
#include "../handlers/Handlers.h"

class DropBehavior : public Behavior{
    enum Stages{
        DRIVE_TO_CENTER = 0,
        DROP_CUBE,
        DRIVE_BACK
    };

    Stages stage = DRIVE_TO_CENTER;

    //The initial x and y from whch we drive
    double x;
    double y;

    int slowDrive = 60;

    public:
        DropBehavior() : Behavior(DROP_BEHAVIOR_TYPE){
            x = OdometryHandler::instance()->getX();
            y = OdometryHandler::instance()->getX();
        }
        bool tick();
};

#endif
