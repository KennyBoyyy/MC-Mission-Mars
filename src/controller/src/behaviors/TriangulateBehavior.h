#ifndef TRIANGULATE_BEHAVIOR_H
#define TRIANGULATE_BEHAVIOR_H

#include "../handlers/Handlers.h"

#include "BehaviorInterface.h"
#include <array>
#include <cmath>

class TriangulateBehavior : public Behavior{
    enum Stages{
        STORE_NAVSAT_VALUES = 0,
    };
    Stages currentStage;

    float radiusBoundary;
    float sumNavX;
    float sumNavY;
    float averageNavX;
    float averageNavY;
    int NavigationListLength;
    int iterationCounter;
    

    public:
        TriangulateBehavior() : Behavior(TRIANGULATE_BEHAVIOR_TYPE){
            currentStage = STORE_NAVSAT_VALUES;
            radiusBoundary = 0.500;
            NavigationListLength = 10000;
        }
        bool tick();
};



#endif // TRIANGULATE_BEHAVIOR_H