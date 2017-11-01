#ifndef HANDLERS_H
#define HANDLERS_H

#include <sensor_msgs/Range.h>


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





















#endif
