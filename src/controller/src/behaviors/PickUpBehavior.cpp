#include "PickUpBehavior.h"

bool PickUpBehavior::tick(){
    switch (currentStage){
        case LOCK_TARGET:
        {
            ClawController::instance()->fingerOpen();
            ClawController::instance()->wristDown();
            targetLocked = false;
            if(TargetHandler::instance()->getNumberOfCubeTags() > 0){
                //get all the tags
                std::vector<Tag> tags = TargetHandler::instance()->getCubeTags();

                // Find closest tag and lock it
                double closest = std::numeric_limits<double>::max();
                int target  = 0;

                //this loop selects the closest visible block to makes goals for it
                for (int i = 0; i < tags.size(); i++)
                {

                  if (tags[i].getID() == 0)
                  {

                    targetLocked = true;

                    //absolute distance to block from camera lens
                    double test = hypot(hypot(tags[i].getPositionX(), tags[i].getPositionY()), tags[i].getPositionZ());

                    if (closest > test)
                    {
                      target = i;
                      closest = test;
                    }
                  }
                }

                if(targetLocked){
                    ///TODO: Explain the trig going on here- blockDistance is c, 0.195 is b; find a
                    blockDistanceFromCamera = hypot(hypot(tags[target].getPositionX(), tags[target].getPositionY()), tags[target].getPositionZ());

                    if ( (blockDistanceFromCamera*blockDistanceFromCamera - 0.195*0.195) > 0 )
                    {
                        blockDistance = sqrt(blockDistanceFromCamera*blockDistanceFromCamera - 0.195*0.195);
                    }
                    else
                    {
                        float epsilon = 0.00001; // A small non-zero positive number
                        blockDistance = epsilon;
                    }

                    cout << "TARGET: Distance to closest: " << blockDistance << endl;

                    //angle to block from bottom center of chassis on the horizontal.
                    blockYawError = atan((tags[target].getPositionX() + cameraOffsetCorrection)/blockDistance)*1.05;

                    cout << "TARGET: Angle to closest:  " << blockYawError << endl;

                    currentStage = TURN_TO_FACE_TARGET;

                    //get the current theta to start counting the turn
                    initTheta = OdometryHandler::instance()->getTheta();
                }

            }
            break;

        }
        case TURN_TO_FACE_TARGET:
        {
            bool leftTurn = true;
            //get current angle
            float currentTheta = OdometryHandler::instance()->getTheta();

            // Check if we have turned to face the cube
            float abs_error = fabs(angles::shortest_angular_distance(currentTheta, initTheta));

            // If angle turned is not within the tolerance
            float abs_blockYaw = fabs(blockYawError);

            if(abs_blockYaw - abs_error <= angleTolerance && abs_blockYaw - abs_error >= -angleTolerance){
                //if within the angle tolerance
                currentStage = DRIVE_TO_PICK_UP;
                DriveController::instance()->stop();

                //get x and y
                initX = OdometryHandler::instance()->getX();
                initY = OdometryHandler::instance()->getY();


            } else {
                float rightWheelMin = DriveController::instance()->getRightMin();
                float leftWheelMin = DriveController::instance()->getLeftMin();

                // Figure out what direction to turn
                if(abs_blockYaw - abs_error > 0){
                    if(blockYawError > 0){
                        leftTurn = false;
                    } else {
                        leftTurn = true;
                    }
                } else {
                    if(blockYawError > 0){
                        leftTurn = true;
                    } else {
                        leftTurn = false;
                    }
                }

                //decide whick way to turn
                if(!leftTurn){
                    //turn right
                    DriveController::instance()->sendDriveCommand(20, -20);
                } else {
                    // turn left
                    DriveController::instance()->sendDriveCommand(-20, 20);
                }
            }
            break;

        }
        case DRIVE_TO_PICK_UP:
        {
            float currX = OdometryHandler::instance()->getX();
            float currY = OdometryHandler::instance()->getY();

            //Drive and count how far we have driven
            float distance = hypot(initX - currX, initY - currY);
            cout << "PICKUP: distance left " << (blockDistance - distance) << " Curr dist: "<<distance<< endl;

            if(blockDistance - distance <= 0){
                currentStage = PICK_UP;
                DriveController::instance()->stop();
                //get x and y
            }else{
                DriveController::instance()->sendDriveCommand(40, 40);
            }

            break;
        }
        case PICK_UP:
        {
            ClawController::instance()->fingerClose();
            sleep(1);
            ClawController::instance()->wristUp();
            sleep(5);
            // check if picked up
            float sonarCenter = SonarHandler::instance()->getSonarCenter();

            if(sonarCenter < 0.12){
                //target was picked up
                ClawController::instance()->wristDown();
                return true;
            } else {
                initX = OdometryHandler::instance()->getX();
                initY = OdometryHandler::instance()->getY();
                currentStage = RETRY;
            }

            break;
        }
        case RETRY:
        {
            ClawController::instance()->wristDown();
            ClawController::instance()->fingerOpen();

            float currX = OdometryHandler::instance()->getX();
            float currY = OdometryHandler::instance()->getY();
            // target was not seen. Drive back and pick up
            //Drive and count how far we have driven
            float distance = hypot(initX - currX, initY - currY);
            cout << "PICKUP: distance drove back " << (distance) << " out of : "<<driveBackDist<< endl;

            if(distance >= driveBackDist){
                currentStage = LOCK_TARGET;
                DriveController::instance()->stop();
            }else{
                DriveController::instance()->sendDriveCommand(-40, -40);
            }
        }


    }

    return false;































}
