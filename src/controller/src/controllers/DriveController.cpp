#include "DriveController.h"

DriveController* DriveController::s_instance = 0;

bool DriveController::goToLocation(float x, float y){
    if(!isInitThetaCalculated){
        //calculate theta to the desired location
        initTheta = atan2(y - OdometryHandler::instance()->getY(), x - OdometryHandler::instance()->getX());
        isInitThetaCalculated = true;
    }

    // Calculate the diffrence between current and desired heading in radians.
    float errorYaw = angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initTheta);
    float errorVel = searchVelocity - (left/255);
    // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
    if (fabs(angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initTheta)) > rotateOnlyAngleTolerance) {
        // rotate but dont drive.
        fastPID(0.0, errorYaw, 0.0, initTheta);
        sendDriveCommand(this->left, this->right);
        return false;
    } else {    //angle is correct. Do drive
        // goal not yet reached drive while maintaining proper heading.
        if (fabs(angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), atan2(y - OdometryHandler::instance()->getY(), x - OdometryHandler::instance()->getX()))) < waypointTolerance) {
            // drive and turn simultaniously
            fastPID(errorVel, errorYaw, searchVelocity, initTheta);
            sendDriveCommand(this->left, this->right);
            return false;
        }
        // goal is reached but desired heading is still wrong turn only
        else if (fabs(angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initTheta)) > finalRotationTolerance) {
            // rotate but dont drive
            fastPID(0.0, errorYaw, 0.0, initTheta);
            sendDriveCommand(this->left, this->right);
            return false;
        }
        else {
          // stopno change
          fastPID(0.0, 0.0, 0.0, 0.0);
          sendDriveCommand(this->left, this->right);
          isInitThetaCalculated = false;
          if(this->left == 0 && this->right == 0){
              return true;
          }else{
              return false;
          }
        }
    }
}

bool DriveController::goToDistance(float distance, float direction){
    if(!isInitLocation){
        //initLocation.theta = direction; //set the dirrection
        initTheta = direction;
        initLocation.x = OdometryHandler::instance()->getX() + (distance * cos(direction));     //Find x
        initLocation.y = OdometryHandler::instance()->getY() + (distance * sin(direction));     //Find y
        isInitLocation = true;
    }



    // Calculate the diffrence between current and desired heading in radians.
    float errorYaw = angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initTheta);
    float errorVel = searchVelocity - (left/255);
    // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
    if (fabs(angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initTheta)) > rotateOnlyAngleTolerance) {
        // rotate but dont drive.
        fastPID(0.0, errorYaw, 0.0, initTheta);
        sendDriveCommand(this->left, this->right);
        return false;
    } else {    //angle is correct. Do drive
        // goal not yet reached drive while maintaining proper heading.
        if (fabs(angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), atan2(initLocation.y - OdometryHandler::instance()->getY(), initLocation.x - OdometryHandler::instance()->getX()))) < waypointTolerance) {
            // drive and turn simultaniously
            fastPID(errorVel, errorYaw, searchVelocity, initTheta);
            sendDriveCommand(this->left, this->right);
            return false;
        }
        // goal is reached but desired heading is still wrong turn only
        else if (fabs(angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initTheta)) > finalRotationTolerance) {
            // rotate but dont drive
            fastPID(0.0, errorYaw, 0.0, initTheta);
            sendDriveCommand(this->left, this->right);
            return false;
        }
        else {
          // stopno change
          fastPID(0.0, 0.0, 0.0, 0.0);
          sendDriveCommand(this->left, this->right);
          if(this->left == 0 && this->right == 0){
              isInitThetaCalculated = false;
              isInitLocation = false;
              return true;
          }else{
              return false;
          }
        }
    }

    return false;


}

bool DriveController::stop(){
    fastPID(0.0, 0.0, 0.0, 0.0);
    sendDriveCommand(this->left, this->right);
    isInitThetaCalculated = false;
    if(this->left == 0 && this->right == 0){
        return true;
        isInitLocation = false;
        isInitTime = false;
    }else{
        return false;
    }
}

int DriveController::spinCounter = 0;
bool DriveController::spinInCircle(float spinVel, int spinTimes){
    if(!isInitLocation){
        initLocation.x = OdometryHandler::instance()->getX();
        initLocation.y = OdometryHandler::instance()->getY();
        initLocation.theta = OdometryHandler::instance()->getTheta();
        isInitLocation = true;
    }

    if(spinCounter <= spinTimes+100){  //if was spinnig less than desired
        sendDriveCommand(left, right);
        if(right < spinVel){
            left -=10;
            right += 10;
        }
        sendDriveCommand(left, right);

        if (fabs(angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initLocation.theta)) > 0.01)
            spinCounter ++;

        return false;
    } else {
        float errorYaw = angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initLocation.theta);
        //we were spinnign for enough time, now level out
        if (fabs(angles::shortest_angular_distance(OdometryHandler::instance()->getTheta(), initLocation.theta)) > rotateOnlyAngleTolerance) {
            // rotate but dont drive.
            fastPID(0.0, errorYaw, 0.0, initLocation.theta);
            sendDriveCommand(this->left, this->right);
            return false;
        } else {
            // stopno change
            fastPID(0.0, 0.0, 0.0, 0.0);
            sendDriveCommand(this->left, this->right);
            isInitThetaCalculated = false;
            if(this->left == 0 && this->right == 0){
                return true;
                isInitLocation = false;
                isInitTime = false;
                spinCounter = 0;
            }else{
                return false;
            }
        }
    }


}



void DriveController::fastPID(float errorVel, float errorYaw , float setPointVel, float setPointYaw) {

  float velOut = fastVelPID.PIDOut(errorVel, setPointVel);
  float yawOut = fastYawPID.PIDOut(errorYaw, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 255;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw) {

      float velOut = slowVelPID.PIDOut(errorVel, setPointVel);
      float yawOut = slowYawPID.PIDOut(errorYaw, setPointYaw);

      int left = velOut - yawOut;
      int right = velOut + yawOut;

      int sat = 255;
      if (left  >  sat) {left  =  sat;}
      if (left  < -sat) {left  = -sat;}
      if (right >  sat) {right =  sat;}
      if (right < -sat) {right = -sat;}

      this->left = left;
      this->right = right;
}

void DriveController::constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw) {

      float velOut = fastVelPID.PIDOut(erroVel, setPointVel);
      float yawOut = fastYawPID.PIDOut(constAngularError, setPointYaw);

      int left = velOut - yawOut;
      int right = velOut + yawOut;

      int sat = 255;
      if (left  >  sat) {left  =  sat;}
      if (left  < -sat) {left  = -sat;}
      if (right >  sat) {right =  sat;}
      if (right < -sat) {right = -sat;}

      this->left = left;
      this->right = right;
}



PIDConfig DriveController::fastVelConfig() {
  PIDConfig config;

  config.Kp = 140;
  config.Ki = 10;
  config.Kd = 0.8;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::fastYawConfig() {
  PIDConfig config;

  config.Kp = 100;
  config.Ki = 10;
  config.Kd = 14;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/2;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::slowVelConfig() {
    PIDConfig config;
  
    config.Kp = 100;
    config.Ki = 8;
    config.Kd = 1.1;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/2;
    config.errorHistLength = 4;
    config.alwaysIntegral = true;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    config.derivativeAlpha = 0.7;
  
    return config;
  
  }
  
  PIDConfig DriveController::slowYawConfig() {
    PIDConfig config;
  
    config.Kp = 70;
    config.Ki = 16;
    config.Kd = 10;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/4;
    config.errorHistLength = 4;
    config.alwaysIntegral = false;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 0;
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/6;
    config.derivativeAlpha = 0.7;
  
    return config;
  
  }
  
  PIDConfig DriveController::constVelConfig() {
    PIDConfig config;
  
    config.Kp = 140;
    config.Ki = 10;
    config.Kd = 0.8;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/2;
    config.errorHistLength = 4;
    config.alwaysIntegral = true;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 320; //gives 127 pwm at 0.4 commandedspeed
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    config.derivativeAlpha = 0.7;
  
    return config;
  
  }
  
  PIDConfig DriveController::constYawConfig() {
    PIDConfig config;
  
    config.Kp = 100;
    config.Ki = 5;
    config.Kd = 1.2;
    config.satUpper = 255;
    config.satLower = -255;
    config.antiWindup = config.satUpper/4;
    config.errorHistLength = 4;
    config.alwaysIntegral = true;
    config.resetOnSetpoint = true;
    config.feedForwardMultiplier = 120;
    config.integralDeadZone = 0.01;
    config.integralErrorHistoryLength = 10000;
    config.integralMax = config.satUpper/2;
    config.derivativeAlpha = 0.6;
  
    return config;
  
  }
