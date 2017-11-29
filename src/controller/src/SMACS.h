#ifndef SMACS_STACK
#define SMACS_STACK

#include<stack>
#include"behaviors/Behaviors.h"
using namespace std;



class SMACS{
    static SMACS* s_instance;
    stack <Behavior*> behaviorStack;

    SMACS(){}

public:
    static SMACS* instance(){
        if(!s_instance)
            s_instance = new SMACS;
        return s_instance;
    }

    void push(Behavior* b){behaviorStack.push(b);}
    void pop(){behaviorStack.pop();}
    bool isEmpty(){return behaviorStack.empty();}
    bool tick();


};







#endif
