#ifndef DRIVE_BEHAVIOR_H
#define DRIVE_BEHAVIOR_H


#include "BehaviorInterface.h"

#include "../controllers/DriveController.h"
#include "../handlers/Handlers.h"


/**
 * @brief The DriveBehavior class - is a simple drive behavior that instructs the controler to
 * drive to x and y.
 */
class DriveBehavior : public Behavior{
    float x=0, y=0;

    public:
        DriveBehavior(float x, float y) : Behavior(DRIVE_BEHAVIOR_TYPE, true){
            this->x = x;
            this->y = y;
        }
        bool tick();
};


/**
 * @brief The CenteDriveBehavior class - Just like the DriveBehavior it simply tells the controller to
 * drive to x and y, however, the only difference is that it turns off center avoid and if while on the
 * way to destination the center is spotted, this behavior will pop from stack and let the next behavior
 * work
 *
 * Note: IT WILL NOT TURN ON THE CENTER AVOID. THAT IS THE JOB OF DROP BEHAVIOR. SO DROP AND THIS SHOUD
 * BE USED TOGETHER.
 */
class CenterDriveBehavior : public Behavior{
    float x=0, y=0;

    public:
        CenterDriveBehavior(float x, float y) : Behavior(DRIVE_BEHAVIOR_TYPE, true){
            this->x = x;
            this->y = y;
        }
        bool tick();
};




#endif
