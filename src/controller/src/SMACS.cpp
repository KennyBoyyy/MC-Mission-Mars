#include "SMACS.h"

SMACS* SMACS::s_instance = 0;

SMACS::SMACS(){}

SMACS* SMACS::instance(){
    if(!s_instance)
        s_instance = new SMACS;
    return s_instance;
}
void SMACS::push(Behavior* b){
    DriveController::instance()->stop();
    behaviorStack.push(b);
}

void SMACS::pop(){
    DriveController::instance()->stop();
    behaviorStack.pop();
}

void SMACS::pushNext(Behavior *b){
    Behavior *hold = behaviorStack.top();
    behaviorStack.pop();
    push(b);
    push(hold);
}


bool SMACS::isEmpty(){return behaviorStack.empty();}

bool SMACS::tick(){
    if(!behaviorStack.empty())
        if(behaviorStack.top()->tick()){
            behaviorStack.pop();
            return true;
        }
    return false;
}
