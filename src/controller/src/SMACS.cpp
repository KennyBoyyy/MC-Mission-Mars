#include "SMACS.h"

SMACS* SMACS::s_instance = 0;

SMACS::SMACS(){}

SMACS* SMACS::instance(){
    if(!s_instance)
        s_instance = new SMACS;
    return s_instance;
}

void SMACS::push(Behavior* b){
    if(b == NULL){
        cout << "STACK: " << "NULL pointer passed to stack" << endl;
        return;
    }

    //lock the stack from being used
    std::lock_guard<std::mutex> guard(stackMutex);
    cout << "STACK: " << "Stack locked"<< endl;

    //if stack is not empty, check if behavior is stackable
    if(!behaviorStack.empty()){
        Behavior *top = behaviorStack.top();
        cout << "STACK: " << "Got top"<< endl;
        if(b->getType() == top->getType()){
            if(!b->isStackable()){
                cout << "STACK: " << "Behavior is not stackable. Was not pushed to stack"<< endl;
                return;
            }
        }
    }

    //stop the robot
    DriveController::instance()->stop();

    //push new behavior
    behaviorStack.push(b);
    cout << "STACK: " << "Pushed to stack"<< endl;

    cout << "STACK: " << "Stack unlocked"<< endl;
}

void SMACS::pop(){
    if(behaviorStack.empty()){
        cout << "STACK: " << "Behavior stack is already empty."<< endl;
        return;
    }

    //lock stack
    std::lock_guard<std::mutex> guard(stackMutex);
    cout << "STACK: " << "Stack locked"<< endl;

    DriveController::instance()->stop();
    cout << "STACK: " << "Poped from stack"<< endl;

    //pop element
    behaviorStack.pop();

    cout << "STACK: " << "Stack unlocked"<< endl;
}

void SMACS::pushNext(Behavior *b){
    if(b == NULL){
        cout << "STACK: " << "NULL pointer passed to stack"<< endl;
        return;
    }

    //lock stack
    std::lock_guard<std::mutex> guard(stackMutex);
    cout << "STACK: " << "Stack locked"<< endl;

    // if stack is not empty, then save top element and pop it to get to the next element
    if(!behaviorStack.empty()){
        Behavior *hold = behaviorStack.top();
        behaviorStack.pop();

        //if stack is not empty, check if second elements is stackable
        if(!behaviorStack.empty()){
            Behavior *top = behaviorStack.top();
            if(b->getType() == top->getType()){
                if(!b->isStackable()){
                    cout << "STACK: " << "Behavior is not stackable. Was not pushed to stack"<< endl;
                    behaviorStack.push(hold);
                    return;
                }
            }
        }

        behaviorStack.push(b);
        behaviorStack.push(hold);
    } else {
        behaviorStack.push(b);
    }

    cout << "STACK: " << "Pushed next behavior to stack."<<endl;
    cout << "STACK: " << "Stack unlocked"<< endl;
}


bool SMACS::isEmpty(){return behaviorStack.empty();}

bool SMACS::tick(){
    if(!behaviorStack.empty())
        if(behaviorStack.top()->tick()){
            pop();
            return true;
        }
    return false;
}






















