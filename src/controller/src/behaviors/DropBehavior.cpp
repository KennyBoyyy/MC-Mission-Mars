#include "DropBehavior.h"

bool DropBehavior::tick(){

    switch(stage){
        case ODOM_TARGET:
        {
            TargetHandler::instance()->setEnabled(false);
            SMACS::instance()->push(new CenterDriveBehavior(0, 0));
            stage = GPS_TARGET;
            break;
        }
        case SEARCH_FOR_CENTER:
        {
            // Check if there is a center that we can see
            if(TargetHandler::instance()->getNumberOfCenterTagsSeen() > 0){
                stage = DROP;
            } else {
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
            }
        }
        case GPS_TARGET:
        {
            // If we see center
            if(TargetHandler::instance()->getNumberOfCenterTagsSeen() > 0){
                SonarHandler::instance()->setEnable(false);
                stage = ASK;
            } else {
                // Set GPS target dive
            }
            stage = SEARCH;
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
        case DROP:
        {
            // drive to center

            // Open fingers
            ClawController::instance()->fingerOpen();

            //Drive back
            DriveController::instance()->sendDriveCommand(-50, -50);
            sleep(1);
            //Wrist up
            ClawController::instance()->wristUp();

            SonarHandler::instance()->setEnable(true);
            TargetHandler::instance()->setEnabled(true);

            return true;
        }

    }

    return false;
}
