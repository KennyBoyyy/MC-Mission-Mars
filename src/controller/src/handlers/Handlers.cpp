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

double SonarHandler::getLeftSonar(){
    return sonarLeft.range;
}


