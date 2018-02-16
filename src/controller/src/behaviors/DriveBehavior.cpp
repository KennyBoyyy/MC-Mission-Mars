#include "DriveBehavior.h"

bool DriveBehavior::tick(){
    cout << "DRIVEBEHAVIOR: "<< "driving to x:"<<this->x<<" y:"<<this->y<<endl;
    if(DriveController::instance()->goToLocation(x, y)){
        return true;
    }
    return false;
}

bool CenterDriveBehavior::tick(){
    // Turn off center avoid
    TargetHandler::instance()->setEnabled(false);

    // If have not seen the center tags
    if(TargetHandler::instance()->getNumberOfCenterTagsSeen() == 0){
        // keep driving
        if(DriveController::instance()->goToLocation(x, y)){
            return true;
        }
    } else {
        // Saw a tag
        return true;
    }
    return false;
}
