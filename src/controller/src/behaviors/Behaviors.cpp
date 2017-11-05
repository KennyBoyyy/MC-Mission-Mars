#include "Behaviors.h"
//=============================================================================================================//
//==============================================SimpleBehavior=================================================//
SimpleBehavior::SimpleBehavior(){}

bool SimpleBehavior::tick(){
    ClawController::instance()->fingerOpen();
    return DriveController::instance()->spinInCircle(255, 5);
}
