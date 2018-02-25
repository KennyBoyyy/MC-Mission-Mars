#ifndef BEHAVIOR_INTERFACE_H
#define BEHAVIOR_INTERFACE_H

#include <sys/time.h>
#include <string>

//Enum specifies the type of the behavior. Needed to make sure that same type behavior is not put on the stack if not allowed
enum Type{ SEARCH_BEHAVIOR_TYPE, AVOID_BEHAVIOR_TYPE, PICKUP_BEHAVIOR_TYPE,
           SEARCH_FOR_DROP_BEHAVIOR_TYPE, DROP_BEHAVIOR_TYPE, DEFAULT_BEHAVIOR_TYPE,
           CALIBRATE_BEHAVIOR_TYPE, DRIVE_BEHAVIOR_TYPE, TRIANGULATE_BEHAVIOR_TYPE};

/**
 * @brief The Behavor class - abstract class that represents a behavior.
 * Has only one method that has to be implemented. The rest is up to the user
 */
class Behavior{
        // The type of the behavior
	Type type;
        // If this behavior can be stackeble on the stack
        bool stackable;

    public:
        Behavior(Type type = DEFAULT_BEHAVIOR_TYPE, bool stackable= false) : type(type), stackable(stackable){}
        virtual bool tick() = 0;    // =0 makes it a pure virtual method, meaning you do not have to implement it in the scope of Behavior
        Type getType(){return type;}
        bool isStackable(){return this->stackable;}

        long millis(){
            struct timeval tp;
            gettimeofday(&tp, 0);
            long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
            return ms;
        }

        int getCurrentBehavior(){
            return type;
        }

};


#endif
