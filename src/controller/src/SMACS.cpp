#include "SMACS.h"

SMACS* SMACS::s_instance = 0;

SMACS::SMACS(){}

SMACS* SMACS::instance(){
    if(!s_instance)
        s_instance = new SMACS;
    return s_instance;
}
void SMACS::push(Behavior* b){behaviorStack.push(b);}
void SMACS::pop(){behaviorStack.pop();}
bool SMACS::isEmpty(){return behaviorStack.empty();}

bool SMACS::tick(){
    if(!behaviorStack.empty())
        if(behaviorStack.top()->tick()){
            behaviorStack.pop();
            return true;
        }
    return false;
}
