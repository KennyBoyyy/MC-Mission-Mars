#include "CalibrateBehavior.h"

bool CalibrateBehavior::tick(){
    switch (currentStage) {
        // Calibration steps
        case FIND_MIN_LEFT_WHEELS:
        {
            // If time is not initialised
            if(!isTimeInit){
                //init start time
                time(&initTime);
                isTimeInit = true;
                //start turning left with current calibration speed
                initTheta = OdometryHandler::instance()->getTheta();
                DriveController::instance()->sendDriveCommand(-leftWheelMin, leftWheelMin);
                cout<<"CALIBRATION: left: "<<leftWheelMin<<endl;

            } else {    //else time is init
                //calculate how many seconds passed
                secSince = difftime(time(NULL), initTime);
                // If enough seconds passed
                if(secSince > secTillNextSpeedIter){
                    cout << "CALIBRATION: "<<"secSince: "<<secSince<<endl;
                    //Check if robot direction changed
                    //calculate the diferance between current theta and initial
                    float currentTheta = OdometryHandler::instance()->getTheta();
                    float abs_error = fabs(angles::shortest_angular_distance(currentTheta, initTheta));

                    //if we turned 5 or more degrees in 5 sec
                    if(abs_error >= angleTolerance){
                        DriveController::instance()->sendDriveCommand(0 ,0);
                        isTimeInit = false;
                        currentStage = FIND_MIN_RIGHT_WHEELS;

                    } else {
                        DriveController::instance()->sendDriveCommand(0 ,0);
                        //else we did not complete the turn. Need to increase the reatio
                        leftWheelMin += iterationInctrease;
                        //reset the init direction
                        isTimeInit = false;

                        cout<<"CALIBRATION: Not enough. Increase left to: "<<leftWheelMin<<endl;
                    }
                }
            }

            break;
        }
        case FIND_MIN_RIGHT_WHEELS:
        {
            // If time is not initialised
            if(!isTimeInit){
                //init start time
                initTime = time(NULL);
                isTimeInit = true;
                //start turning left with current calibration speed
                initTheta = OdometryHandler::instance()->getTheta();
                DriveController::instance()->sendDriveCommand(rightWheelMin, -rightWheelMin);
                cout<<"CALIBRATION: right: "<<rightWheelMin<<endl;
            } else {    //else time is init
                //calculate how many seconds passed
                secSince = difftime(time(NULL), initTime);
                // If enough seconds passed
                if(secSince > secTillNextSpeedIter){
                    cout << "CALIBRATION: "<<"secSince: "<<secSince<<endl;
                    //Check if robot direction changed
                    //calculate the diferance between current theta and initial
                    float currentTheta = OdometryHandler::instance()->getTheta();
                    float abs_error = fabs(angles::shortest_angular_distance(currentTheta, initTheta));

                    //if we turned 5 or more degrees in 5 sec
                    if(abs_error >= angleTolerance){
                        DriveController::instance()->sendDriveCommand(0 ,0);
                        isTimeInit = false;
                        currentStage = FIND_RATIO;

                    } else {
                        DriveController::instance()->sendDriveCommand(0 ,0);
                        //else we did not complete the turn. Need to increase the reatio
                        rightWheelMin += iterationInctrease;
                        //reset the init direction
                        isTimeInit = false;

                        cout<<"CALIBRATION: Not enough. Increase right to: "<<rightWheelMin<<endl;

                    }
                }
            }

            break;
        }
        case FIND_RATIO:
        {
            cout <<"CALIBRATION: left: " <<leftWheelMin<<" right: "<<rightWheelMin<<endl;
            DriveController::instance()->stop();
            DriveController::instance()->setLeftRightMin(leftWheelMin, rightWheelMin);

            SMACS::instance()->pushNext(new SearchBehavior());
            SMACS::instance()->pushNext(new SearchBehavior());
            return true;
        }

        default:
            return true;
    }

    return false;
}





























