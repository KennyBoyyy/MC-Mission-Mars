#include "SearchBehavior.h"

//=============================================================================================================//
//============================================SearchBehavior===================================================//
bool SearchBehavior::tick(){
    if(first){
        nextPoint();
    } else {
        if(DriveController::instance()->goToDistance(distance, theta)){
            nextPoint();
        }
    }


    return false;
}


void SearchBehavior::nextPoint(){
    if(first){
        theta = OdometryHandler::instance()->getTheta() + M_PI;
        distance = 1;
        first = false;
    } else if(second){
        theta = OdometryHandler::instance()->getTheta() + M_PI_2;
        distance = 1;
        second = false;
    } else if(third){
        theta = OdometryHandler::instance()->getTheta() + M_PI_2;
        distance = 2;
        third = false;
        iterCount ++;
    } else {
        if(iterCount >= 2){
            distance += 0.5;
            iterCount = 0;
        }
        iterCount ++;
        theta = OdometryHandler::instance()->getTheta() + M_PI_2;
    }


}


























