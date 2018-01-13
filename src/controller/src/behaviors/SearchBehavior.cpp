#include "SearchBehavior.h"

//=============================================================================================================//
//============================================SearchBehavior===================================================//
bool SearchBehavior::tick(){
    if(first){
        nextPoint();
    } else {
        cout<<"SEARCH: Drive"<<endl;
        if(DriveController::instance()->goToLocation(x, y)){
            cout<<"SEARCH: NEW Point"<<endl;
            nextPoint();
        }
    }


    return false;
}


void SearchBehavior::nextPoint(){
    if(first){
        theta = OdometryHandler::instance()->getTheta() + M_PI;
        distance = 1;
        x = OdometryHandler::instance()->getX() + ((distance) * cos(theta));
        y = OdometryHandler::instance()->getY() + ((distance) * sin(theta));
        first = false;
    } else if(second){
        theta = OdometryHandler::instance()->getTheta() + M_PI_2;
        distance = 2;
        x = OdometryHandler::instance()->getX() + ((distance) * cos(theta));
        y = OdometryHandler::instance()->getY() + ((distance) * sin(theta));
        second = false;
    } else if(third){
        theta = OdometryHandler::instance()->getTheta() + M_PI_2;
        distance = 4;
        x = OdometryHandler::instance()->getX() + ((distance) * cos(theta));
        y = OdometryHandler::instance()->getY() + ((distance) * sin(theta));
        third = false;
        iterCount ++;
    } else {
        if(iterCount >= 2){
            distance += 0.5;
            iterCount = 0;
        }
        iterCount ++;
        theta = OdometryHandler::instance()->getTheta() + M_PI_2;
        x = OdometryHandler::instance()->getX() + ((distance) * cos(theta));
        y = OdometryHandler::instance()->getY() + ((distance) * sin(theta));
    }


}


























