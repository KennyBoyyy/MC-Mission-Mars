#include "ClawController.h"

ClawController* ClawController::s_instance = 0;

bool ClawController::wristDown(){
    wristPublisher.publish(wristDownAngle);
    return true;
}

bool ClawController::wristUp(){
    wristPublisher.publish(wristUpAngle);
    return true;
}

bool ClawController::fingerOpen(){
    fingerPublisher.publish(fingerOpenAngle);
    return true;
}

bool ClawController::fingerClose(){
    fingerPublisher.publish(fingerCloseAngle);
    return true;
}
