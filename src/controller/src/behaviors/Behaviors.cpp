#include "Behaviors.h"
//=============================================================================================================//
//==============================================SimpleBehavior=================================================//
SimpleBehavior::SimpleBehavior(){}

bool SimpleBehavior::tick(){
    ClawController::instance()->fingerOpen();
    ClawController::instance()->wristDown();
}
