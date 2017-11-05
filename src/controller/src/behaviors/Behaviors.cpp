#include "Behaviors.h"
//=============================================================================================================//
//==============================================SimpleBehavior=================================================//
SimpleBehavior::SimpleBehavior(ros::Publisher& test){
    this->test = test;
}

bool SimpleBehavior::tick(){
    //DriveController::instance()->sendDriveCommand(20, 20);
}
