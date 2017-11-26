#ifndef BEHAVIORS_H
#define BEHAVIORS_H

#include "ros/ros.h"
#include "../controllers/DriveController.h"
#include "../controllers/ClawController.h"
#include "std_msgs/Float32.h"

/**
 * @brief The Behavor class - abstract class that represents a behavior.
 * Has only one method that has to be implemented. The rest is up to the user
 */
class Behavior{
    public:
        virtual bool tick() = 0;    // =0 makes it a pure virtual method, meaning you do not have to implement it in the scope of Behavior
};



/**
 * @brief The SimpleBehavior class - Simple testing behavior that probably wont work
 */
class SimpleBehavior: public Behavior{
    ros::Publisher test;

    public:
        SimpleBehavior();
        bool tick();

};

class SquarePathBehavior: public Behavior
{
    ros::Publisher test;

    public:
        SquarePathBehavior();
        bool tick();

};

class PickUpBehavior: public Behavior
{
    ros::Publisher test;

    public:
        PickUpBehavior();
        bool tick();
};

class SearchBehavior: public Behavior{
    bool first = true;
    bool second = true;
    float theta;
    float distance;
    public:
        bool tick();
};

#endif
