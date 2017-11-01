#include "Handlers.h"

//==============================================================================//
//                         Sonar handler methods                                //
//==============================================================================//
//declare and init the instance to null
SonarHandler *SonarHandler::s_instance = 0;

void SonarHandler::setDisableAvoid(bool &isDisable){
    isAvoidDisabled = isDisable;
}

bool const &SonarHandler::isDisabled(){
    return isAvoidDisabled;
}

void SonarHandler::handleLeft(const sensor_msgs::Range::ConstPtr& sonarLeft){
    this->sonarLeft.range = sonarLeft->range;
}

void SonarHandler::handleCenter(const sensor_msgs::Range::ConstPtr& sonarCenter){
    this->sonarCenter.range = sonarCenter->range;
}

void SonarHandler::handleRight(const sensor_msgs::Range::ConstPtr& sonarRight){
    this->sonarRight.range = sonarRight->range;
}

//==============================================================================//
//==============================================================================//

