#include "AvoidBehavior.h"


bool AvoidBehavior::tick(){
    float minColisionDistanse = SonarHandler::instance()->getMinDistance();
    float left = SonarHandler::instance()->getSonarLeft();
    float center = SonarHandler::instance()->getSonarCenter();
    float right = SonarHandler::instance()->getSonarRight();

    switch (stage) {
        case WAIT:
        {
            //if we haven't started counting the time, start
            if(!isTimeInit){
                time(&initTime);
                isTimeInit = true;
            } else {
                // Find how many seconds passed
                int secSince = difftime(time(NULL), initTime);

                //If enough time passed
                if(secSince >= waitTime){
                    //Check if we are still blocked
                    float left = SonarHandler::instance()->getSonarLeft();
                    float right = SonarHandler::instance()->getSonarRight();
                    float center = SonarHandler::instance()->getSonarCenter();

                    float minDistance = SonarHandler::instance()->getMinDistance();

                    //Id path is clear, return true to signal the completion of avoid behavior
                    if(center>minDistance && left>minDistance && right>minDistance){
                        return true;
                    } else {
                        //Path is not clear. Have to move
                        isTimeInit = false;
                        stage = TURN;
                    }
                }
            }
            break;
        }
        case TURN:
        {

            //See what direction is better to turn to
            //if left is blocked more than right
            if(left < right){
                //turn right until center and right are clear

                //if right is blocked but it is not too bad keep turning
                float minTurnRight = DriveController::instance()->getRightMin();

                //if we are not in the emergency stop point
                //and if obstacle on center is less than meter away
                if(right < minColisionDistanse || center <= clearDistance || left <= emergencyStop){
                    DriveController::instance()->turnRight(minTurnRight + 20);
                }else{
                    //else center is 1 meter clear. we can drive forward
                    DriveController::instance()->stop();
                    stage = DRIVE;
                }


                //if right is super blocked, stop and do wait again.
                if(right <= emergencyStop){
                    //maybe drive backwards
                    break;
                }



            } else {
                // else left is more clear so have to turn left
                //turn left until center and left are clear

                //if left is blocked but it is not too bad keep turning
                float minTurnleft = DriveController::instance()->getLeftMin();

                //if we are not in the emergency stop point
                //and if obstacle on center is less than meter away
                if(right <= emergencyStop || center <= clearDistance || left < minColisionDistanse){
                    DriveController::instance()->turnLeft(minTurnleft + 10);
                }else{
                    //else center is 1 meter clear. we can drive forward
                    DriveController::instance()->stop();
                    directionToDrive = OdometryHandler::instance()->getTheta();
                    stage = DRIVE;
                }


                //if left is super blocked, stop and do wait again.
                if(left <= emergencyStop){
                    //Maybe drive backwards
                    break;
                }
            }

            break;
        }
        case DRIVE:
        {
            // Check if we can drive forward
            if(left <= emergencyStop || right <= emergencyStop || center < minColisionDistanse){
                DriveController::instance()->stop();
                stage = TURN;
            }

            // Drive forvard and if finished return true
            if(DriveController::instance()->goToDistance(1, directionToDrive)){
                DriveController::instance()->stop();
                return true;
            } else {
                return false;
            }



            break;
        }

        default:
            return true;
    }

    return false;
}
