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
   // DriveController::instance()->goToLocation(5, 0);
   // DriveController::instance()->goToLocation(5, 5);
   // DriveController::instance()->goToLocation(0, 5);
    return DriveController::instance()->goToLocation(-5, 5);
}

//=============================================================================================================//
//============================================PickUpBehavior===============================================//

PickUpBehavior::PickUpBehavior(){}

bool PickUpBehavior::tick()
{
   
    return ClawController::instance()->wristUp();
}