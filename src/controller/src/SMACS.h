#ifndef SMACS_STACK_H
#define SMACS_STACK_H

#include "behaviors/BehaviorInterface.h"
#include "controllers/DriveController.h"
#include <stack>
#include <mutex>
#include <iostream>

using namespace std;

class Behavior;

class SMACS{
    static SMACS* s_instance;
    // Stack that holds behaviors
    std::stack <Behavior*> behaviorStack;

    // Mutex makes SMACS thread safe
    std::mutex stackMutex;

    SMACS();
    bool locked = false;

public:
    static SMACS* instance();

    //pushes on top of the stack
    void push(Behavior* b);
    //pops from top of the stack
    void pop();
    /*
     * push an element to be next after element is popped.
     * EX: we have behaviors 1, 2 in stack in that order. 1 is on top.
     * pushNext(3) will do this 1, 3, 2. Meaing 3 will be the next behavior
     * to be executed after top is popped.
     */
    void pushNext(Behavior *b);
    bool isEmpty();
    bool tick();
};







#endif