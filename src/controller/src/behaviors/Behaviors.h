/**
  Behaviors.h is the include file for MainLoop
*/
#ifndef BEHAVIORS_H
#define BEHAVIORS_H

//include SearchController
#include "SearchBehavior.h"


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
