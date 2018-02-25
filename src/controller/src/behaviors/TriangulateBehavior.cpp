#include "TriangulateBehavior.h"

bool TriangulateBehavior::tick(){
    switch(currentStage) {
        case STORE_NAVSAT_VALUES:
        {
            float NavigationListArray[3][NavigationListLength];
            float currentNavSeq = NavigationHandler::instance()->getNavSeq();
            float currentNavX = NavigationHandler::instance()->getNavX();
            float currentNavY = NavigationHandler::instance()->getNavY();

            iterationCounter = 0;
            while (iterationCounter < NavigationListLength)
            {
                if (pow(radiusBoundary,2) - pow((currentNavX - (-1.0)),2) <= pow((currentNavY - (0.0)),2))  // Testing Achilles starting at xinit = -1 and yinit = 0
                {   
                    NavigationListArray[1][iterationCounter] = currentNavSeq;
                    NavigationListArray[2][iterationCounter] = currentNavX;
                    NavigationListArray[3][iterationCounter] = currentNavY;
                    iterationCounter++;
                }
                cout<<"NAVSAT:"<<"Seq: "<< currentNavSeq<< "x: "<< NavigationListArray[2][iterationCounter]<< "y: "<< NavigationListArray[3][iterationCounter]<< endl; 
            }
            for (iterationCounter = 0; iterationCounter < NavigationListLength; iterationCounter++)
            {
                sumNavX += NavigationListArray[2][iterationCounter];
                sumNavY += NavigationListArray[3][iterationCounter];
            }

            averageNavX = sumNavX/NavigationListLength;
            averageNavY = sumNavY/NavigationListLength;

            cout<<"avgNavX:"<<averageNavX<<"avgNavY:"<<averageNavY<<endl;
            break;
        }

        default:
            return true;
    }

    return false;
}