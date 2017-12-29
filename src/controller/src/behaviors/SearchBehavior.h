#ifndef SEARCH_BEHAVIOR_H
#define SEARCH_BEHAVIOR_H

#include "BehaviorInterface.h"

class SearchBehavior: public Behavior{
    bool first = true;
    bool second = true;
    bool third = true;
    float theta;
    float distance;

    float x;
    float y;

    int iterCount = 0;
    public:
        SearchBehavior() : Behavior(SearchBehaviorType){}
        bool tick();
        void nextPoint();
};



#endif
