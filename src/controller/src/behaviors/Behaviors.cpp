#include "Behaviors.h"
//=============================================================================================================//
//==============================================SimpleBehavior=================================================//
SimpleBehavior::SimpleBehavior(){}

bool SimpleBehavior::tick(){
    ClawController::instance()->wristDown();
    ClawController::instance()->fingerOpen();
    return DriveController::instance()->spinInCircle(255, 5);
}

//=============================================================================================================//
//============================================SquarePathBehavior===============================================//

SquarePathBehavior::SquarePathBehavior(){}

bool SquarePathBehavior::tick()
{
    return DriveController::instance()->goToDistance(2, OdometryHandler::instance()->getTheta() + M_PI_2);
}

//=============================================================================================================//
//============================================SearchBehavior===================================================//
bool SearchBehavior::tick(){
    if(first){
        theta = OdometryHandler::instance()->getTheta() + M_PI;
        distance = 0.5;
        if(DriveController::instance()->goToDistance(distance, theta)){
            theta = OdometryHandler::instance()->getTheta() + M_PI_2;
            distance = 1.5;
            first = false;

        }
    } else if(second){
        if(DriveController::instance()->goToDistance(distance, theta)){
            second = false;
            theta = OdometryHandler::instance()->getTheta() + M_PI_2;
            distance = 3;
        }
    } else {
        if(DriveController::instance()->goToDistance(distance, theta)){
            theta = OdometryHandler::instance()->getTheta() + M_PI_2;
            distance += 0.5;
            return false;
        }
    }


    return false;
}
