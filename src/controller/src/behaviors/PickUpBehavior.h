#ifndef PICKUP_BEHAVIOR_H
#define PICKUP_BEHAVIOR_H

#include "../handlers/Handlers.h"
#include "../SMACS.h"
#include "../handlers/Tag.h"

#include "../controllers/ClawController.h"

#include "BehaviorInterface.h"

#include <vector>

#include <time.h>

class PickUpBehavior : public Behavior{
    enum Stages{
        LOCK_TARGET = 0,
        TURN_TO_FACE_TARGET,
        DRIVE_TO_PICK_UP,
        PICK_UP,
        DONE
    };
    Stages currentStage;

    bool targetLocked;

    float cameraOffsetCorrection = 0.023; //meters
    float blockDistanceFromCamera = 0;
    float blockDistance = 0;
    float blockYawError = 0;

    float initTheta = 0;
    float initX = 0;
    float initY = 0;
    float angleTolerance = 0.0175;



    public:
        PickUpBehavior() : Behavior(PICKUP_BEHAVIOR_TYPE){
            currentStage = LOCK_TARGET;

            targetLocked = false;
        }

        bool tick();
};




#endif
