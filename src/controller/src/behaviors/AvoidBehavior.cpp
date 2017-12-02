#include "AvoidBehavior.h"

AvoidBehavior::AvoidBehavior() : Behavior(AvoidBehaviorType){
    waitTime = 10; // 5 seconds of wait time before moving on
    initTime = time(NULL); //get current time
}

bool AvoidBehavior::tick(){
    if(difftime(time(NULL), initTime) >= waitTime){
        DriveController::instance()->stop();    //stop the robot;
        return true;
    }

    return false;
}
