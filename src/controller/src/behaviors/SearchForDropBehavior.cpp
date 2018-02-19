#include "SearchForDropBehavior.h"

bool SearchForDropBehavior::tick(){
    if(TargetHandler::instance()->getHasCube()){
        //turn on sonars for avoid
        SonarHandler::instance()->setEnable(true);
        //turn off camera for center avoid and cube pick up
        TargetHandler::instance()->setEnabled(false);
        switch(stage){
            case ODOM_TARGET:
            {
                TargetHandler::instance()->setEnabled(false);
                if(DriveController::instance()->goToLocation(0, 0)){
                    stage = SEARCH_FOR_CENTER;
                }

                break;
            }
            case SEARCH_FOR_CENTER:
            {
                //If we do not see any tags yet. Try to drive around
                //If this is our first search try
                if(searchTry == 0){
                    // Drive one meter forward
                    if(DriveController::instance()->goToDistance(1, OdometryHandler::instance()->getTheta())){
                        searchTry++;
                    }
                } else if(searchTry == 1){
                    // If second try, figure out what to do.
                    stage = GPS_TARGET;
                }

                break;
            }
            case GPS_TARGET:
            {
                searchTry = 0;
                if(DriveController::instance()->goToLocation(0, 0)){
                    stage = SEARCH_FOR_CENTER;
                }
                break;
            }
            case SEARCH:
            {
                stage = ASK;
                break;
            }
            case ASK:
            {
                SonarHandler::instance()->setEnable(false);
                stage = DROP;
                break;
            }
        }
    } else {
        return true;
    }

    return false;
}
