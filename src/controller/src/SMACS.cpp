#include "SMACS.h"

SMACS* SMACS::s_instance = 0;

bool SMACS::tick(){
    if(!behaviorStack.empty())
        if(behaviorStack.top()->tick()){
            behaviorStack.pop();
            return true;
        }
    return false;
}
