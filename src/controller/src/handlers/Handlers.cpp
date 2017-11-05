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


//==============================================================================//
//                          Odom handler methods                                //
//==============================================================================//
OdometryHandler* OdometryHandler::s_instance = 0;

void OdometryHandler::handle(const nav_msgs::Odometry::ConstPtr &message){
    this->currentLocation.x = message->pose.pose.position.x;
    this->currentLocation.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
}

