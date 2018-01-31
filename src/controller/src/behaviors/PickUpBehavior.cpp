#include "PickUpBehavior.h"

bool PickUpBehavior::tick(){
    switch (currentStage){
        case LOCK_TARGET:
        {
            SonarHandler::instance()->setEnable(false);
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

                    if(!precisionDrive)
                        currentStage = TURN_TO_FACE_TARGET;
                    else
                        currentStage = PRECISION_TURN;

                    //get the current theta to start counting the turn
                    initTheta = OdometryHandler::instance()->getTheta();
                }

            } else {
                DriveController::instance()->stop();
                SonarHandler::instance()->setEnable(true);
                ClawController::instance()->fingerClose();
                ClawController::instance()->wristUp();
                return true;
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

            //if within the angle tolerance
            if(abs_blockYaw - abs_error <= angleTolerance){
                currentStage = DRIVE_TO_PICK_UP;
                DriveController::instance()->stop();

                //get x and y
                initX = OdometryHandler::instance()->getX();
                initY = OdometryHandler::instance()->getY();
            } else {
                float rightWheelMin = DriveController::instance()->getRightMin();
                float leftWheelMin = DriveController::instance()->getLeftMin();

                if (blockYawError < 0){
                    //turn left
                    if(abs_blockYaw - abs_error > 0)
                        DriveController::instance()->sendDriveCommand(-leftWheelMin, rightWheelMin);
                    else
                        DriveController::instance()->sendDriveCommand(leftWheelMin, -rightWheelMin);

                } else {
                    //trun right
                    if(abs_blockYaw - abs_error > 0)
                        DriveController::instance()->sendDriveCommand(leftWheelMin, -rightWheelMin);
                    else
                        DriveController::instance()->sendDriveCommand(-leftWheelMin, rightWheelMin);
                }
            }

            break;

        }
        case DRIVE_TO_PICK_UP:
        {
            float currX = OdometryHandler::instance()->getX();
            float currY = OdometryHandler::instance()->getY();

            ClawController::instance()->fingerOpen();
            ClawController::instance()->wristDown();

            //Drive and count how far we have driven
            float distance = hypot(initX - currX, initY - currY);
            cout << "PICKUP: distance left " << (blockDistance - distance) << " Curr dist: "<<distance<< endl;

            if(blockDistance - distance <= 0.35){
                currentStage = LOCK_TARGET;
                precisionDrive = true;
                DriveController::instance()->stop();

            }else{
                DriveController::instance()->sendDriveCommand(driveSpeed, driveSpeed);
            }

            break;
        }
        case PRECISION_TURN:
        {
            //turn again once closer to the cube
            bool leftTurn = true;
            //get current angle
            float currentTheta = OdometryHandler::instance()->getTheta();

            // Check if we have turned to face the cube
            float abs_error = fabs(angles::shortest_angular_distance(currentTheta, initTheta));

            // If angle turned is not within the tolerance
            float abs_blockYaw = fabs(blockYawError);

            if(abs_blockYaw - abs_error - 0.175 <= angleTolerance){
                //if within the angle tolerance
                currentStage = PRECISION_DRIVE;
                DriveController::instance()->stop();

                //get x and y
                initX = OdometryHandler::instance()->getX();
                initY = OdometryHandler::instance()->getY();


            } else {
                float rightWheelMin = DriveController::instance()->getRightMin();
                float leftWheelMin = DriveController::instance()->getLeftMin();

                if (blockYawError < 0){
                    //turn left
                    if(abs_blockYaw - abs_error - 0.175 > 0)
                        DriveController::instance()->sendDriveCommand(-leftWheelMin, rightWheelMin);
                    else
                        DriveController::instance()->sendDriveCommand(leftWheelMin, -rightWheelMin);

                } else {
                    //trun right
                    if(abs_blockYaw - abs_error - 0.175 > 0)
                        DriveController::instance()->sendDriveCommand(leftWheelMin, -rightWheelMin);
                    else
                        DriveController::instance()->sendDriveCommand(-leftWheelMin, rightWheelMin);
                }
            }

            break;
        }
        case PRECISION_DRIVE:
        {
            float currX = OdometryHandler::instance()->getX();
            float currY = OdometryHandler::instance()->getY();

            ClawController::instance()->fingerOpen();
            ClawController::instance()->wristDown();

            //Drive and count how far we have driven
            float distance = hypot(initX - currX, initY - currY);
            cout << "PICKUP: distance left " << (blockDistance - distance) << " Curr dist: "<<distance<< endl;

            if(blockDistance - distance <= 0.035){
                currentStage = PICK_UP;

                DriveController::instance()->stop();
            }else{
                DriveController::instance()->sendDriveCommand(driveSpeed, driveSpeed);
            }

            break;
        }
        case PICK_UP:
        {
            ClawController::instance()->fingerClose();
            sleep(1);
            ClawController::instance()->wristUp();


            if(!wait(2)){
                // check if picked up
                float sonarCenter = SonarHandler::instance()->getSonarCenter();
                cout << "PICKUP: Center sonar: " <<sonarCenter<< endl;

                if(sonarCenter < 0.12){
                    //TODO: maybe add a camera block seen chack by checking how far is the picked up block from camera
                    //target was picked up
                    ClawController::instance()->wristDownWithCube();
                    TargetHandler::instance()->setEnabled(false);
                    SonarHandler::instance()->setEnable(true);
                    currentStage = DROP;
                } else {
                    initX = OdometryHandler::instance()->getX();
                    initY = OdometryHandler::instance()->getY();
                    currentStage = RETRY;
                }
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
                int numberOftags = TargetHandler::instance()->getNumberOfCenterTagsSeen();
                if(numberOftags > 0){
                    ClawController::instance()->wristUp();
                    ClawController::instance()->fingerClose();
                    precisionDrive = false;
                    currentStage = LOCK_TARGET;
                    DriveController::instance()->stop();
                } else {
                    // no tragets are seen after drive back. try to turn.
                    float lastYaw = TargetHandler::instance()->getLastSeenBlockError();
                    if(lastYaw < 0){
                        DriveController::instance()->sendDriveCommand(-driveSpeed, driveSpeed);
                        sleep(0.5);
                        DriveController::instance()->stop();
                    } else {
                         DriveController::instance()->sendDriveCommand(driveSpeed, -driveSpeed);
                         sleep(0.5);
                         DriveController::instance()->stop();
                    }

                    ClawController::instance()->wristUp();
                    ClawController::instance()->fingerClose();
                    precisionDrive = false;
                    currentStage = LOCK_TARGET;
                    DriveController::instance()->stop();
                }
            }else{
                DriveController::instance()->sendDriveCommand(-driveSpeed, -driveSpeed);
            }

            break;
        }
        case DROP:
        {
            //Get current x and y
            float x = OdometryHandler::instance()->getX();
            float y = OdometryHandler::instance()->getY();

            //Put return behavior in the stack
            SMACS::instance()->pushNext(new DriveBehavior(x, y));
            //Put drop behavior to the stack
            SMACS::instance()->pushNext(new DropBehavior());

            //return true to pop pick up from stack and execute DropBehavior()
            return true;
        }

    }

    return false;

}



bool PickUpBehavior::wait(int sec){
    if(!waiting){
        waiting = true;
        time(&initTime);
        return true;
    } else {
        time(&currTime);
        int secSince = difftime(currTime, initTime);
        cout << "PICKUP: Waiting: "<<secSince<<" out of "<<sec<<endl;
        if(difftime(currTime, initTime) >= sec){
            waiting = false;
            return false;
        }
    }

    return true;
}






























