#include "DriveBehavior.h"

bool DriveBehavior::tick(){
    if(DriveController::instance()->goToLocation(x, y)){
        return true;
    }
    return false;
}
