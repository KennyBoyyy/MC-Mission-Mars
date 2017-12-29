#include "ClawController.h"

ClawController* ClawController::s_instance = 0;

ClawController::ClawController() {
    fingerOpenAngle.data =  M_PI_2;
    fingerCloseAngle.data = 0.0;
    wristDownAngle.data = M_PI_2;
    wristUpAngle.data = 0.0;
}

ClawController* ClawController::instance(){
    if(!s_instance)
        s_instance = new ClawController;
    return s_instance;
}

void ClawController::registerPublishers(ros::Publisher& fingerPublisher, ros::Publisher wristPublisher){
    this->fingerPublisher = fingerPublisher;
    this->wristPublisher = wristPublisher;
}

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
