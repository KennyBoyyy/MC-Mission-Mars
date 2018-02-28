#include "AvoidBehavior.h"


bool AvoidBehavior::tick(){
    float minColisionDistanse = SonarHandler::instance()->getMinDistance();
    float left = SonarHandler::instance()->getSonarLeft();
    float center = SonarHandler::instance()->getSonarCenter();
    float right = SonarHandler::instance()->getSonarRight();
    float minDistance = minColisionDistanse;
    switch (stage) {
        case WAIT:
        {
            cout<<"AVOID: waiting"<<endl;
            //if we haven't started counting the time, start
            if(!isTimeInit){
                time(&initTime);
                isTimeInit = true;
            } else {
                // Find how many seconds passed
                int secSince = difftime(time(NULL), initTime);

                //If enough time passed
                if(secSince >= waitTime){
                    //If path is clear, return true to signal the completion of avoid behavior
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
            cout<<"AVOID: Turning"<<endl;
            if(!turnLock){
                if(left < right){
                    isLeftTurn = false;
                } else {
                    isLeftTurn = true;
                }
                turnLock = true;
            }

            if(switchTurnCount < 3){
                if(isLeftTurn){
                    // if left turn and left is super blocked switch to right
                    if(left <= emergencyStop){
                        isLeftTurn = false;
                        switchTurnCount++;
                    }

                } else {
                    // if right turn and right is super blocked switch to left
                    if(right <= emergencyStop){
                        isLeftTurn = true;
                        switchTurnCount++;
                    }
                }


                //See what direction is better to turn to
                //if left is blocked more than right
                if(!isLeftTurn){
                    cout<<"AVOID: Trying right"<<endl;
                    //turn right until center and right are clear

                    //if right is blocked but it is not too bad keep turning
                    float minTurnRight = DriveController::instance()->getRightMin();

                    //if we are not in the emergency stop point
                    //and if obstacle on center is less than meter away
                    if(right <= minColisionDistanse|| center <= clearDistance || left <= minColisionDistanse+0.05){
                        DriveController::instance()->turnRight(minTurnRight + 20);
                    }else{
                        //else center is 1 meter clear. we can drive forward
                        DriveController::instance()->stop();
                        stage = DRIVE;
                        turnLock = false;
                        switchTurnCount = 0;
                        isTimeInit = false;
                    }


//                    //if right is super blocked, stop and do wait again.
//                    if(right <= emergencyStop){
//                        //maybe drive backwards
//                        break;
//                    }



                } else {
                    cout<<"AVOID: Trying left"<<endl;
                    // else left is more clear so have to turn left
                    //turn left until center and left are clear

                    //if left is blocked but it is not too bad keep turning
                    float minTurnleft = DriveController::instance()->getLeftMin();

                    //if we are not in the emergency stop point
                    //and if obstacle on center is less than meter away
                    if(right <= minColisionDistanse+0.05 || center <= clearDistance || left <= minColisionDistanse){
                        DriveController::instance()->turnLeft(minTurnleft + 20);
                    }else{
                        //else center is 1 meter clear. we can drive forward
                        DriveController::instance()->stop();
                        directionToDrive = OdometryHandler::instance()->getTheta();
                        stage = DRIVE;
                        turnLock = false;
                        switchTurnCount = 0;
                        isTimeInit = false;
                    }


//                    //if left is super blocked, stop and do wait again.
//                    if(left <= emergencyStop){
//                        //Maybe drive backwards
//                        break;
//                    }
                }

            } else {
                cout<<"AVOID: Driving back. cant turn"<<endl;
                //if we haven't started counting the time, start
                if(!isTimeInit){
                    time(&initTime);
                    isTimeInit = true;
                } else {
                    float minTurnleft = DriveController::instance()->getLeftMin();

                    // Find how many seconds passed
                    int secSince = difftime(time(NULL), initTime);
                    DriveController::instance()->sendDriveCommand(-minTurnleft, -minTurnleft);
                    //If enough time passed
                    if(secSince >= 2){

                        turnLock = false;
                        switchTurnCount = 0;
                        //Path is not clear. Have to move
                        isTimeInit = false;

                    }
                }
                break;
            }

            break;
        }
        case DRIVE:
        {
            // Drive forvard and if finished return true
            if(DriveController::instance()->goToDistance(0.5, directionToDrive)){
                DriveController::instance()->stop();
                return true;
            } else {
                cout<<"AVOID: Drinving 0.5 meter "<<minColisionDistanse<<" "<<left<<endl;
                // Check if we can drive forward
                if(left < minColisionDistanse || right < minColisionDistanse || center < minColisionDistanse){
                    DriveController::instance()->stop();
                    stage = TURN;
                    break;
                }
                return false;
            }



            break;
        }

        default:
            return true;
    }

    return false;
}







