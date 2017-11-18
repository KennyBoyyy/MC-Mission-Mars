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
    return DriveController::instance()->goToLocation(-5, 5);
}

