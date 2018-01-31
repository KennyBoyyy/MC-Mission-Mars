#ifndef DRIVE_BEHAVIOR_H
#define DRIVE_BEHAVIOR_H


#include "BehaviorInterface.h"

#include "../controllers/DriveController.h"

class DriveBehavior : public Behavior{
    double x, y;

    public:
        DriveBehavior(double driveX, double driveY) : Behavior(DRIVE_BEHAVIOR_TYPE, true){
            cout << "DRIVEBEHAVIOR: "<< "Setting x:"<<x<<" y:"<<y<<endl;
            x = driveX;
            y = driveY;
        }
        bool tick();
};







#endif
