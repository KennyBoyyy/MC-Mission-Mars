#ifndef SMACS_STACK_H
#define SMACS_STACK_H

#include "behaviors/BehaviorInterface.h"
#include<stack>
//using namespace std;

class Behavior;

class SMACS{
    static SMACS* s_instance;
    std::stack <Behavior*> behaviorStack;
    SMACS();

public:
    static SMACS* instance();

    void push(Behavior* b);
    void pop();
    void pushNext(Behavior *b);
    bool isEmpty();
    bool tick();
    Behavior* top()const {return behaviorStack.top();}


};







#endif
