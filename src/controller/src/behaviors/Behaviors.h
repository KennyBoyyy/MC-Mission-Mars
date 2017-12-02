/**
  Behaviors.h is the include file for MainLoop
*/
#ifndef BEHAVIORS_H
#define BEHAVIORS_H


//include SearchController
#include "SearchBehavior.h"

#include "ros/ros.h"
#include "../controllers/DriveController.h"
#include "../controllers/ClawController.h"
#include "std_msgs/Float32.h"

enum Type{ SearchBehaviorType, AvoidBehaviorType, PickUpBehaviorType, TestBehaviorType };



/**
 * @brief The SimpleBehavior class - Simple testing behavior that probably wont work
 */
class SimpleBehavior: public Behavior{

    public:
        SimpleBehavior();
        bool tick();

};

class SquarePathBehavior: public Behavior
{

    public:
        SquarePathBehavior();
        bool tick();

};

class PickUpBehavior: public Behavior
{

    public:
        PickUpBehavior();
        bool tick();
};


#endif



































