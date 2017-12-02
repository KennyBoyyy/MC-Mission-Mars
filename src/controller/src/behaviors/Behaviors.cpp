#include "Behaviors.h"


//=============================================================================================================//
//==============================================SimpleBehavior=================================================//
SimpleBehavior::SimpleBehavior() : Behavior(TestBehaviorType){}

bool SimpleBehavior::tick(){
    ClawController::instance()->wristDown();
    ClawController::instance()->fingerOpen();
    return false;
}

//=============================================================================================================//
//============================================SquarePathBehavior===============================================//

SquarePathBehavior::SquarePathBehavior() : Behavior(TestBehaviorType){}

bool SquarePathBehavior::tick()
{
    return DriveController::instance()->goToLocation(5, 5);
}

