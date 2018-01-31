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
            // Turn off sonar because we found center now we wait
            SonarHandler::instance()->setEnable(false);
            stage = ASK;
            break;
        }
        case ASK:
        {
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
            return true;
        }

    }

    return false;
}
