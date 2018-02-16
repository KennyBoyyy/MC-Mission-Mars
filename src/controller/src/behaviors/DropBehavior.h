#ifndef DROP_BEHAVIOR_H
#define DROP_BEHAVIOR_H

#include "BehaviorInterface.h"

#include "../controllers/DriveController.h"
#include "../controllers/ClawController.h"
#include "../handlers/Handlers.h"
#include "DriveBehavior.h"
#include "../SMACS.h"

class DropBehavior : public Behavior{
    enum Stages{
        ODOM_TARGET = 0,
        SEARCH_FOR_CENTER,
        GPS_TARGET,
        SEARCH,
        ASK,
        DROP
    };

   Stages stage = ODOM_TARGET;
   int searchTry = 0;

    public:
        DropBehavior() : Behavior(DROP_BEHAVIOR_TYPE){

        }

        bool tick();
};














#endif
