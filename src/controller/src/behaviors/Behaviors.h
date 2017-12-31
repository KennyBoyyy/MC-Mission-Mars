/**
  Behaviors.h is the include file for MainLoop
*/
#ifndef BEHAVIORS_H
#define BEHAVIORS_H

#include "BehaviorInterface.h"

//include SearchController
#include "SearchBehavior.h"
#include "CalibrateBehavior.h"

#include "ros/ros.h"
#include "../controllers/DriveController.h"
#include "../controllers/ClawController.h"
#include "std_msgs/Float32.h"


/**
 * @brief The SimpleBehavior class - Simple testing behavior that probably wont work
 */
//class SimpleBehavior: public Behavior{

//    public:
//        SimpleBehavior() : Behavior(TestBehaviorType){}
//        bool tick();

//};

//class SquarePathBehavior: public Behavior{

//    public:
//        SquarePathBehavior(): Behavior(TestBehaviorType){}
//        bool tick();

//};

//class PickUpBehavior: public Behavior{

//    public:
//        PickUpBehavior(): Behavior(TestBehaviorType){}
//        bool tick();
//};


#endif



































