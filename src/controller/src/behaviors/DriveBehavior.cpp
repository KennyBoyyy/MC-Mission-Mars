#include "DriveBehavior.h"

bool DriveBehavior::tick(){
    cout << "DRIVEBEHAVIOR: "<< "driving to x:"<<this->x<<" y:"<<this->y<<endl;
    if(DriveController::instance()->goToLocation(x, y)){
        return true;
    }
    return false;
}
