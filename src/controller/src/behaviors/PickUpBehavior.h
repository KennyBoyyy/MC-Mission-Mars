#ifndef PICKUP_BEHAVIOR_H
#define PICKUP_BEHAVIOR_H

#include "../handlers/Handlers.h"
#include "../SMACS.h"
#include "../handlers/Tag.h"

#include "../controllers/ClawController.h"

#include "BehaviorInterface.h"
#include "DriveBehavior.h"
#include "DropBehavior.h"

#include <vector>

#include <time.h>

class PickUpBehavior : public Behavior{
    enum Stages{
        LOCK_TARGET = 0,
        TURN_TO_FACE_TARGET,
        DRIVE_TO_PICK_UP,
        PRECISION_TURN,
        PRECISION_DRIVE,
        PICK_UP,
        RETRY,
        DONE,
        DROP
    };
    Stages currentStage;

    bool targetLocked;
    bool precisionDrive = false;

    float cameraOffsetCorrection = 0.023; //meters
    float blockDistanceFromCamera = 0;
    float blockDistance = 0;
    float blockYawError = 0;

    float initTheta = 0;
    float initX = 0;
    float initY = 0;
    float angleTolerance = 0.0175;

    float driveBackDist = 0.25;

    int driveSpeed = 40;

    bool waiting;
    time_t initTime;
    time_t currTime;


    public:
        PickUpBehavior() : Behavior(PICKUP_BEHAVIOR_TYPE){
            currentStage = LOCK_TARGET;

            targetLocked = false;
        }

        bool tick();

        bool wait(int sec);
};




#endif
