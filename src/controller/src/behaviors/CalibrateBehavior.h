#ifndef CALIBRATE_BEHAVIOR_H
#define CALIBRATE_BEHAVIOR_H

#include "../handlers/Handlers.h"
#include "../SMACS.h"


#include "BehaviorInterface.h"

#include "SearchBehavior.h"





#include <time.h>


class CalibrateBehavior: public Behavior{

    enum Stages {
        FIND_MIN_LEFT_WHEELS = 0,
        FIND_MIN_RIGHT_WHEELS,
        RUTURN_TO_POSITION,
        FIND_RATIO

    };
    Stages currentStage;

    float initialAngle;

    int leftWheelMin;
    int rightWheelMin;
    int iterationInctrease;

    time_t initTime;
    bool isTimeInit;
    int secSince;
    int secTillNextSpeedIter;
    float angleTolerance;
    float initTheta;

    bool isRetunrSet;
    float returnTheta;


    public:
        CalibrateBehavior() : Behavior(CALIBRATE_BEHAVIOR_TYPE){
            currentStage = FIND_MIN_LEFT_WHEELS;
            secSince = 0;
            isTimeInit = false;
            secTillNextSpeedIter = 3;
            initTheta = OdometryHandler::instance()->getTheta();

            isRetunrSet = false;
            returnTheta = initTheta;

            angleTolerance = 0.175;

            rightWheelMin = 30;
            leftWheelMin = 30;

            iterationInctrease =5;

        }

        bool tick();

};






#endif
