#include "CalibrateBehavior.h"


bool CalibrateBehavior::tick(){

    if(!isRetunrSet){
        returnTheta = OdometryHandler::instance()->getTheta();
        isRetunrSet = true;
        lastMillis = millis();
    }

    switch (currentStage) {
        // Calibration steps
        case FIND_MIN_LEFT_TURN:
        {
            DriveController::instance()->sendDriveCommand(leftWheelMinNeg, rightWheelMinPos);
            //Left positive, right negative
            //if enought time passed
            if(millis() - lastMillis > millisToNextIncrease){
                cout<<"CALIBRATION: "<<" turning left: "<< leftWheelMinNeg <<", " << rightWheelMinPos << endl;
                int left = fabs(EncoderHandler::instance()->getEncoderLeft());
                int right = fabs(EncoderHandler::instance()->getEncoderRight());
                cout<<"CALIBRATION: "<<"e_l: "<< left << "e_r: " << right << endl;
                //if current encoders not equal the desired value
                if(fabs(left - encoderTickStopPoint) > 10 && !leftTurnLeftWheel){
                    //if it is too fast slow down. If slow speed up
                    if(left < encoderTickStopPoint){
                        leftWheelMinNeg -= iterationInctrease;
                    } else {
                        leftWheelMinNeg += iterationInctrease;
                    }
                } else {
                    leftTurnLeftWheel = true;
                }

                // Check right wheel encoders
                if(fabs(right - encoderTickStopPoint) > 10 && !rightTurnLeftWheel){
                    if(right < encoderTickStopPoint){
                        rightWheelMinPos += iterationInctrease;
                    } else {
                        rightWheelMinPos -= iterationInctrease;
                    }
                } else {
                    leftTurnLeftWheel = true;
                }

                lastMillis = millis();
            }

            if(leftTurnLeftWheel && leftTurnRightWheel){
                currentStage == FIND_MIN_RIGHT_TURN;
            }
            break;
        }
        case FIND_MIN_RIGHT_TURN:
        {
            if(millis() - lastMillis > millisToNextIncrease){

            }
            break;
        }
        case RUTURN_TO_POSITION:
        {
            float currentTheta = OdometryHandler::instance()->getTheta();
            float abs_error = fabs(angles::shortest_angular_distance(currentTheta, initTheta));

            if(abs_error >= finalAngleTolerance){
                DriveController::instance()->sendDriveCommand(0 ,0);
                currentStage = FIND_RATIO;
            }

            if (error < 0){
                DriveController::instance()->sendDriveCommand(leftWheelMinPos, -rightWheelMinNeg);
            } else {
                DriveController::instance()->sendDriveCommand(-leftWheelMinNeg, rightWheelMinPos);
            }

            cout << "CALIBRATION: " << "Error is :"<< error << endl;

            break;
        }

        case FIND_RATIO:
        {
            cout <<"CALIBRATION: left: " <<leftWheelMinPos<<" right: "<<rightWheelMinPos<<endl;
            DriveController::instance()->stop();
            cout <<"CALIBRATION: Setting mins "<<endl;
            DriveController::instance()->setLeftRightMin(leftWheelMinPos, rightWheelMinPos);
            cout <<"CALIBRATION: Putting search "<<endl;
            SMACS::instance()->pushNext(new SearchBehavior());
            cout <<"CALIBRATION: Done "<<endl;
            return true;
        }

        default:
            return true;
    }

    return false;
}





























