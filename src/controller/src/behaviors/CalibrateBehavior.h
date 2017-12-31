#ifndef CALIBRATE_BEHAVIOR_H
#define CALIBRATE_BEHAVIOR_H

#include "Behaviors.h"
#include "time.h"

class CalibrateBehavior: public Behavior{
    enum Stages {
        FIND_MIN_LEFT_WHEELS = 0,
        FIND_MIN_RIGHT_WHEELS,
        FIND_RATIO

    };
    Stages currentStage;

    int leftWheelMin;
    int rightWheelMin;
    int iterationInctrease;

    time_t initTime;
    bool isTimeInit;
    int secSince;
    int secTillNextSpeedIter;
    float angleTolerance;
    float initTheta;
    float returnTheta;


    public:
        CalibrateBehavior() : Behavior(CALIBRATE_BEHAVIOR_TYPE){
            currentStage = FIND_MIN_LEFT_WHEELS;
            secSince = 0;
            isTimeInit = false;
            secTillNextSpeedIter = 3;
            initTheta = OdometryHandler::instance()->getTheta();
            returnTheta = initTheta;
            angleTolerance = 0.175;

            rightWheelMin = 5;
            leftWheelMin = 5;

            iterationInctrease =5;

        }

        bool tick();

};






#endif
