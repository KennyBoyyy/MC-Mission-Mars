#ifndef CALIBRATE_BEHAVIOR_H
#define CALIBRATE_BEHAVIOR_H

#include "../handlers/Handlers.h"
#include "../SMACS.h"


#include "BehaviorInterface.h"

#include "SearchBehavior.h"

#include <sys/time.h>


class CalibrateBehavior: public Behavior{

    enum Stages {
        FIND_MIN_LEFT_TURN = 0,
        FIND_MIN_RIGHT_TURN,
        RUTURN_TO_POSITION,
        FIND_RATIO

    };
    Stages currentStage;

    bool leftTurnLeftWheel;
    bool leftTurnRightWheel;

    bool rightTurnLeftWheel;
    bool rightTurnRightWheel;

    float initialAngle;

    int leftWheelMinPos;
    int leftWheelMinNeg;

    int rightWheelMinPos;
    int rightWheelMinNeg;

    int iterationInctrease;

    long lastMillis;
    int millisToNextIncrease;
    float angleTolerance;
    float finalAngleTolerance;
    float initTheta;

    bool isRetunrSet;
    float returnTheta;
    float error = 0;

    int encoderTickStopPoint;
    int lastLeftE;
    int lastRightE;


    public:
        CalibrateBehavior() : Behavior(CALIBRATE_BEHAVIOR_TYPE){
            leftTurnLeftWheel = false;
            leftTurnRightWheel = false;

            rightTurnLeftWheel = false;
            rightTurnRightWheel = false;

            currentStage = FIND_MIN_LEFT_TURN;
            lastMillis = 0;
            millisToNextIncrease = 200;
            initTheta = OdometryHandler::instance()->getTheta();

            isRetunrSet = false;
            returnTheta = initTheta;

            finalAngleTolerance = 0.0175;

            leftWheelMinPos= 20;
            leftWheelMinNeg = -20;

            rightWheelMinPos = 20;
            rightWheelMinNeg = -20;

            iterationInctrease =5;
            encoderTickStopPoint = 200;

            lastLeftE = 0;
            lastRightE = 0;

        }

        bool tick();

        // Returns the current time in millis
        long millis(){
            struct timeval tp;
            gettimeofday(&tp, 0);
            long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
            return ms;
        }

};






#endif
