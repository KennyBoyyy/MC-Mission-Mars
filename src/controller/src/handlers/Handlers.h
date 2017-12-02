#ifndef HANDLERS_H
#define HANDLERS_H

//ROS messages
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

//ROS libraries
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include <vector>

#include "../Point.h"
#include "Tag.h"
#include "../behaviors/Behaviors.h"
#include "../SMACS.h"


/**
 * This executable file will contain all the handlers and their functionality
 */

/**
 * @brief The SonarHandler class - this singleton class will be responsible for storing and
 * handling the sonar events
 */
class SonarHandler{
    double minDistance; //if sonar is this distance from obstacle then put avoid behavior on stack
    bool isAvoidEnabled; //if obstacle avoid is off we will not do anything about obstacles. We will ignore
    static SonarHandler *s_instance; //static instance of class

    //values to hold sonar callbacks
    sensor_msgs::Range sonarLeft;
    sensor_msgs::Range sonarCenter;
    sensor_msgs::Range sonarRight;

    //private by default
    SonarHandler();

public:
    static SonarHandler* instance();
    void setEnable(const bool &isEnabled);
    bool const &isEnabled();

    //handlers
    void handleLeft(const sensor_msgs::Range::ConstPtr& sonarLeft);
    void handleCenter(const sensor_msgs::Range::ConstPtr& sonarCenter);
    void handleRight(const sensor_msgs::Range::ConstPtr& sonarRight);


    double getSonarLeft();
    double getSonarCenter();
    double getSonarRight();
};




class OdometryHandler{
    Point currentLocation;
    float linearVelocity;
    float angularVelocity;
    static OdometryHandler* s_instance;

    OdometryHandler();

public:
    static OdometryHandler* instance();

    void handle(const nav_msgs::Odometry::ConstPtr& message);
    float getTheta(){return currentLocation.theta;}
    float getX(){return currentLocation.x;}
    float getY(){return currentLocation.y;}
    float getLinear(){return linearVelocity;}
    float getAngular(){return angularVelocity;}

};


class TargetHandler{
    static TargetHandler* s_instance;
    std::vector<Tag> tagList;

    TargetHandler();

public:
    static TargetHandler* instance();

    void handle(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
    int numberOfTagsSeen();

};




















#endif
