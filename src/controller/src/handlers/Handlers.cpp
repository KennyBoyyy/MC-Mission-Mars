#include "Handlers.h"

//==============================================================================//
//                         Sonar handler methods                                //
//==============================================================================//
//declare and init the instance to null
SonarHandler *SonarHandler::s_instance = 0;

SonarHandler::SonarHandler(){
    minDistance = 1; //min distance to call an avoid.
    isAvoidEnabled = true;
}

SonarHandler* SonarHandler::instance(){
    if (!s_instance)
      s_instance = new SonarHandler;
    return s_instance;
}

void SonarHandler::setEnable(const bool &isEnabled){
    isAvoidEnabled = isEnabled;
}

bool const &SonarHandler::isEnabled(){
    return isAvoidEnabled;
}

void SonarHandler::handleLeft(const sensor_msgs::Range::ConstPtr& sonarLeft){
    this->sonarLeft.range = sonarLeft->range;
    if(sonarLeft->range <= minDistance && isAvoidEnabled){
        SMACS::instance()->push(new SimpleBehavior());
        isAvoidEnabled = false;
    }
}

void SonarHandler::handleCenter(const sensor_msgs::Range::ConstPtr& sonarCenter){
    this->sonarCenter.range = sonarCenter->range;
    if(sonarCenter->range <= minDistance && isAvoidEnabled){
        SMACS::instance()->push(new SimpleBehavior());
        isAvoidEnabled = false;
    }
}

void SonarHandler::handleRight(const sensor_msgs::Range::ConstPtr& sonarRight){
    this->sonarRight.range = sonarRight->range;
    if(sonarRight->range <= minDistance && isAvoidEnabled){
        SMACS::instance()->push(new SimpleBehavior());
        isAvoidEnabled = false;
    }
}

double SonarHandler::getSonarLeft(){return sonarLeft.range;}
double SonarHandler::getSonarCenter(){return sonarCenter.range;}
double SonarHandler::getSonarRight(){return sonarRight.range;}

//==============================================================================//
//==============================================================================//


//==============================================================================//
//                          Odom handler methods                                //
//==============================================================================//
OdometryHandler *OdometryHandler::s_instance = 0;

OdometryHandler::OdometryHandler(){}

OdometryHandler* OdometryHandler::instance(){
    if(!s_instance)
        s_instance = new OdometryHandler;
    return s_instance;
}

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

float OdometryHandler::getTheta(){return currentLocation.theta;}
float OdometryHandler::getX(){return currentLocation.x;}
float OdometryHandler::getY(){return currentLocation.y;}
//==============================================================================//
//==============================================================================//


//==============================================================================//
//                          Targets handler methods                             //
//==============================================================================//
TargetHandler *TargetHandler::s_instance = 0;

TargetHandler::TargetHandler(){}

TargetHandler* TargetHandler::instance() {
    if(!s_instance)
        s_instance = new TargetHandler;
    return s_instance;
}

void TargetHandler::handle(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message)
{
    if (message->detections.size() > 0) {
        std::vector<Tag> tags;
    
        for (int i = 0; i < message->detections.size(); i++) {
    
          // Package up the ROS AprilTag data into our own type that does not rely on ROS.
          Tag loc;
          loc.setID( message->detections[i].id );
    
          // Pass the position of the AprilTag
          geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
          loc.setPosition(std::make_tuple(tagPose.pose.position.x, tagPose.pose.position.y,tagPose.pose.position.z));
    
          // Pass the orientation of the AprilTag
          loc.setOrientation( ::boost::math::quaternion<float>( tagPose.pose.orientation.x,
                                    tagPose.pose.orientation.y,
                                    tagPose.pose.orientation.z,
                                    tagPose.pose.orientation.w ) );
          tags.push_back(loc);
        }
        tagList = tags;
    } else {
        tagList.clear();
    }
}

int TargetHandler::numberOfTagsSeen(){return tagList.size();}
//==============================================================================//
//==============================================================================//
