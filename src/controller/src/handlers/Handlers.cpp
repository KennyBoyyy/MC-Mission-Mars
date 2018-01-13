#include "Handlers.h"

//==============================================================================//
//                         Sonar handler methods                                //
//==============================================================================//
//declare and init the instance to null
SonarHandler *SonarHandler::s_instance = 0;

SonarHandler::SonarHandler(){
    minDistance = 0.5; //min distance to call an avoid.
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
    //If avoid is on and sonar is too close
    float currentRange = sonarLeft->range;
    if(isAvoidEnabled && currentRange <= minDistance) {
        SMACS::instance()->push(new AvoidBehavior());
    }
    this->sonarLeft.range = sonarLeft->range;
}

void SonarHandler::handleCenter(const sensor_msgs::Range::ConstPtr& sonarCenter){
    float currentRange = sonarCenter->range;
    if(isAvoidEnabled && currentRange <= minDistance) {
        SMACS::instance()->push(new AvoidBehavior());
    }
    this->sonarCenter.range = sonarCenter->range;
}

void SonarHandler::handleRight(const sensor_msgs::Range::ConstPtr& sonarRight){
    float currentRange = sonarRight->range;
    if(isAvoidEnabled && currentRange <= minDistance) {
        SMACS::instance()->push(new AvoidBehavior());
    }
    this->sonarRight.range = sonarRight->range;
}

float SonarHandler::getSonarLeft(){return sonarLeft.range;}
float SonarHandler::getSonarCenter(){return sonarCenter.range;}
float SonarHandler::getSonarRight(){return sonarRight.range;}

float SonarHandler::getMinDistance(){return minDistance;}

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

    linearVelocity = message->twist.twist.linear.x;
    angularVelocity = message->twist.twist.angular.z;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
}

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
    std::lock_guard<std::mutex> guard(instanceMutex);
    if (message->detections.size() > 0) {
        std::vector<Tag> cubeTags;
        std::vector<Tag> centerTags;

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

          //if cube
          if(message->detections[i].id == 0) {
              cubeTags.push_back(loc);
          } else {
              centerTags.push_back(loc);
          }
        }
        cubeTagsList = cubeTags;
        centerTagsList = centerTags;

        if(isHandlerOn)
            SMACS::instance()->push(new PickUpBehavior());


    } else {
        cubeTagsList.clear();
        centerTagsList.clear();
    }
}

int TargetHandler::getNumberOfCubeTags(){
    std::lock_guard<std::mutex> guard(instanceMutex);
    return cubeTagsList.size();
}
int TargetHandler::getNumberOfCenterTagsSeen(){
    std::lock_guard<std::mutex> guard(instanceMutex);
    return centerTagsList.size();
}

std::vector<Tag> TargetHandler::getCubeTags(){
    std::lock_guard<std::mutex> guard(instanceMutex);
    return cubeTagsList;
}
std::vector<Tag> TargetHandler::getCenterTags(){
    std::lock_guard<std::mutex> guard(instanceMutex);
    return centerTagsList;
}

//float TargetHandler::getDistanceToClosestCube(){

//    std::vector<Tag> tags = cubeTagsList;

//    bool targetLocked = false;
//    float blockDistance = 0;

//    // Find closest tag and lock it
//    double closest = std::numeric_limits<double>::max();
//    int target  = 0;

//    //this loop selects the closest visible block to makes goals for it
//    for (int i = 0; i < tags.size(); i++)
//    {

//      if (tags[i].getID() == 0)
//      {

//        targetLocked = true;

//        //absolute distance to block from camera lens
//        double test = hypot(hypot(tags[i].getPositionX(), tags[i].getPositionY()), tags[i].getPositionZ());

//        if (closest > test)
//        {
//          target = i;
//          closest = test;
//        }
//      }
//    }

//    if(targetLocked){
//        ///TODO: Explain the trig going on here- blockDistance is c, 0.195 is b; find a
//        float blockDistanceFromCamera = hypot(hypot(tags[target].getPositionX(), tags[target].getPositionY()), tags[target].getPositionZ());

//        if ( (blockDistanceFromCamera*blockDistanceFromCamera - 0.195*0.195) > 0 )
//        {
//            blockDistance = sqrt(blockDistanceFromCamera*blockDistanceFromCamera - 0.195*0.195);
//        }
//        else
//        {
//            float epsilon = 0.00001; // A small non-zero positive number
//            blockDistance = epsilon;
//        }
//    }

//    cout << "TARGET: Distance to closest: " << blockDistance << endl;
//}

//float TargetHandler::getYawToClosestCube(){
//    //angle to block from bottom center of chassis on the horizontal.
//    blockYawError = atan((tags[target].getPositionX() + cameraOffsetCorrection)/blockDistance)*1.05;

//    cout << "TARGET: Angle to closest:  " << blockYawError << endl;
//}

//==============================================================================//
//==============================================================================//


























