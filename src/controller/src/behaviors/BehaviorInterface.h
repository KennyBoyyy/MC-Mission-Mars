#ifndef BEHAVIOR_INTERFACE_H
#define BEHAVIOR_INTERFACE_H

#include "../controllers/DriveController.h"
#include "../controllers/ClawController.h"
#include "../handlers/Handlers.h"


enum Type{ SearchBehaviorType, AvoidBehaviorType, PickUpBehaviorType, TestBehaviorType };

/**
 * @brief The Behavor class - abstract class that represents a behavior.
 * Has only one method that has to be implemented. The rest is up to the user
 */
class Behavior{
	Type type;

    public:
    	Behavior(Type type) : type(type){}
        virtual bool tick() = 0;    // =0 makes it a pure virtual method, meaning you do not have to implement it in the scope of Behavior
        Type getType(){return type;}
        void setType(Type type){this->type = type;}
};


#endif