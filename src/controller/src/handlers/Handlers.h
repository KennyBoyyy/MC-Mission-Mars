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

/**
 * This executable file will contain all the handlers and their functionality
 */

/**
 * @brief The SonarHandler class - this singleton class will be responsible for storing and
 * handling the sonar events
 */
class SonarHandler{
    double minDistance; //if sonar is this distance from obstacle then put avoid behavior on stack
    bool isAvoidDisabled; //if obstacle avoid is off we will not do anything about obstacles. We will ignore
    static SonarHandler *s_instance; //static instance of class

    //values to hold sonar callbacks
    sensor_msgs::Range sonarLeft;
    sensor_msgs::Range sonarCenter;
    sensor_msgs::Range sonarRight;

    //private by default
    SonarHandler(){
        minDistance = 0.5; //min distance to call an avoid.
        isAvoidDisabled = false;
    }

public:
    static SonarHandler* instance(){
        if (!s_instance)
          s_instance = new SonarHandler;
        return s_instance;
    }
    void setDisableAvoid(bool &isDisable);
    bool const &isDisabled();

    //handlers
    void handleLeft(const sensor_msgs::Range::ConstPtr& sonarLeft);
    void handleCenter(const sensor_msgs::Range::ConstPtr& sonarCenter);
    void handleRight(const sensor_msgs::Range::ConstPtr& sonarRight);


    double getSonarLeft(){return sonarLeft.range;}
    double getSonarCenter(){return sonarCenter.range;}
    double getSonarRight(){return sonarRight.range;}
};




class OdometryHandler{
    Point currentLocation;
    static OdometryHandler* s_instance;

    OdometryHandler(){}

public:
    static OdometryHandler* instance(){
        if(!s_instance)
            s_instance = new OdometryHandler;
        return s_instance;
    }

    void handle(const nav_msgs::Odometry::ConstPtr& message);
    float getTheta(){return currentLocation.theta;}
    float getX(){return currentLocation.x;}
    float getY(){return currentLocation.y;}
};


class TargetHandler{
    static TargetHandler* s_instance;
    std::vector<Tag> tagList;

    TargetHandler(){}

public:
    static TargetHandler* instance() {
        if(!s_instance)
            s_instance = new TargetHandler;
        return s_instance;
    }

    void handle(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message);
    int numberOfTagsSeen(){return tagList.size();}

};



















#endif
