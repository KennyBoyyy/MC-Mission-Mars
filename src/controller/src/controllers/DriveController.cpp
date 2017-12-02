#include "DriveController.h"


DriveController* DriveController::s_instance = 0;

DriveController* DriveController::instance(){
    if (!s_instance)
      s_instance = new DriveController;
    return s_instance;
}

void DriveController::registerDrivePublisher(ros::Publisher& drivePublisher){
    this->drivePublisher = drivePublisher;
}


bool DriveController::goToLocation(float x, float y){
    //If the requested coords are the same as before
    //Means we are still driving to the same location as requested before
    if(currentDrive.x == x && currentDrive.y == y){
        currentLocation.x = OdometryHandler::instance()->getX();
        currentLocation.y = OdometryHandler::instance()->getY();
        currentLocation.theta = OdometryHandler::instance()->getTheta();
        linear = OdometryHandler::instance()->getLinear();
        switch(stateMachineState){
            case STATE_MACHINE_ROTATE:
            {
                // Calculate angle between currentLocation.theta and waypoints.front().theta
                // Rotate left or right depending on sign of angle
                // Stay in this state until angle is minimized
                currentDrive.theta = atan2(currentDrive.y - currentLocation.y, currentDrive.x - currentLocation.x);

                // Calculate the diffrence between current and desired heading in radians.
                float errorYaw = angles::shortest_angular_distance(currentLocation.theta, currentDrive.theta);

                //Calculate absolute value of angle
                float abs_error = fabs(angles::shortest_angular_distance(currentLocation.theta, currentDrive.theta));

                // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
                if (abs_error > rotateOnlyAngleTolerance){
                    fastPID(0.0, errorYaw, 0.0, currentLocation.theta);
                    break;
                } else {
                //move to differential drive step
                stateMachineState = STATE_MACHINE_SKID_STEER;
                //fall through on purpose.
                }
            }
            case STATE_MACHINE_SKID_STEER:
            {
                // Calculate angle between currentLocation.y and currentDrive.Y
                // Drive forward
                // Stay in this state until angle is at least PI/2

                // calculate the distance between current and desired heading in radians
                currentDrive.theta = atan2(currentDrive.y - currentLocation.y, currentDrive.x - currentLocation.x);
                float errorYaw = angles::shortest_angular_distance(currentLocation.theta, currentDrive.theta);
                float distance = hypot(currentDrive.x - currentLocation.x, currentDrive.y - currentLocation.y);


                // goal not yet reached drive while maintaining proper heading.
                if (fabs(errorYaw) < M_PI_2 &&  distance > waypointTolerance){
                    //cout << "linear velocity:  " << linearVelocity << endl;
                    fastPID((searchVelocity-linear) ,errorYaw, searchVelocity, currentLocation.theta);
                } else {
                    // stopno change
                    stop();
                    // move back to transform step
                    stateMachineState = STATE_MACHINE_ROTATE;
                    return true;
                }
                break;
            }
            default:
            {
                break;
            }
        }
        sendDriveCommand(left, right);
    } else {
        //reset the drive controller and drive to new location
        resetDriveController(x, y);
    }

    return false;

}


bool DriveController::goToDistance(float distance, float direction){
    if(this->distance == distance && this->direction == direction){
        currentLocation.x = OdometryHandler::instance()->getX();
        currentLocation.y = OdometryHandler::instance()->getY();
        currentLocation.theta = OdometryHandler::instance()->getTheta();
        linear = OdometryHandler::instance()->getLinear();
        switch(stateMachineState){
            case STATE_MACHINE_ROTATE:
            {
                // Calculate angle between currentLocation.theta and waypoints.front().theta
                // Rotate left or right depending on sign of angle
                // Stay in this state until angle is minimized
                currentDrive.theta = atan2(currentDrive.y - currentLocation.y, currentDrive.x - currentLocation.x);

                // Calculate the diffrence between current and desired heading in radians.
                float errorYaw = angles::shortest_angular_distance(currentLocation.theta, currentDrive.theta);

                //Calculate absolute value of angle
                float abs_error = fabs(angles::shortest_angular_distance(currentLocation.theta, currentDrive.theta));

                // If angle > rotateOnlyAngleTolerance radians rotate but dont drive forward.
                if (abs_error > rotateOnlyAngleTolerance){
                    fastPID(0.0, errorYaw, 0.0, currentLocation.theta);
                    break;
                } else {
                //move to differential drive step
                stateMachineState = STATE_MACHINE_SKID_STEER;
                //fall through on purpose.
                }
            }
            case STATE_MACHINE_SKID_STEER:
            {
                // Calculate angle between currentLocation.y and currentDrive.Y
                // Drive forward
                // Stay in this state until angle is at least PI/2

                // calculate the distance between current and desired heading in radians
                currentDrive.theta = atan2(currentDrive.y - currentLocation.y, currentDrive.x - currentLocation.x);
                float errorYaw = angles::shortest_angular_distance(currentLocation.theta, currentDrive.theta);
                float distance = hypot(currentDrive.x - currentLocation.x, currentDrive.y - currentLocation.y);


                // goal not yet reached drive while maintaining proper heading.
                if (fabs(errorYaw) < M_PI_2 &&  distance > waypointTolerance){
                    //cout << "linear velocity:  " << linearVelocity << endl;
                    fastPID((searchVelocity-linear) ,errorYaw, searchVelocity, currentLocation.theta);
                } else {
                    // stopno change
                    stop();
                    // move back to transform step
                    stateMachineState = STATE_MACHINE_ROTATE;
                    return true;
                }
                break;
            }
            default:
            {
                break;
            }
        }
        sendDriveCommand(left, right);
    } else {
        //values are different from last time, so change to new values
        this->distance = distance;
        this->direction = direction;

        currentDrive.x = OdometryHandler::instance()->getX() + (distance * cos(direction));     //Find x
        currentDrive.y = OdometryHandler::instance()->getY() + (distance * sin(direction));     //Find y
        currentDrive.theta = direction;
    }
    return false;

}

void DriveController::resetDriveController(float x, float y){
    left = 0;
    right = 0;
    stateMachineState = STATE_MACHINE_ROTATE;
    currentDrive.x = x;
    currentDrive.y = y;
    currentDrive.theta = atan2(y - OdometryHandler::instance()->getY(), x - OdometryHandler::instance()->getX());
    sendDriveCommand(left, right);
}


bool DriveController::stop(){
    left = 0;
    right = 0;
    sendDriveCommand(left, right);
}

void DriveController::sendDriveCommand(double left, double right){
    velocity.linear.x = left;
    velocity.angular.z = right;

    // publish the drive commands
    drivePublisher.publish(velocity);
}











void DriveController::fastPID(float errorVel, float errorYaw , float setPointVel, float setPointYaw)
{

  // cout << "PID FAST" << endl;

  float velOut = fastVelPID.PIDOut(errorVel, setPointVel);
  float yawOut = fastYawPID.PIDOut(errorYaw, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw)
{
  //cout << "PID SLOW" << endl;

  float velOut = slowVelPID.PIDOut(errorVel, setPointVel);
  float yawOut = slowYawPID.PIDOut(errorYaw, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}

void DriveController::constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw)
{

  //cout << "PID CONST" << endl;

  float velOut = constVelPID.PIDOut(erroVel, setPointVel);
  float yawOut = constYawPID.PIDOut(constAngularError, setPointYaw);

  int left = velOut - yawOut;
  int right = velOut + yawOut;

  int sat = 180;
  if (left  >  sat) {left  =  sat;}
  if (left  < -sat) {left  = -sat;}
  if (right >  sat) {right =  sat;}
  if (right < -sat) {right = -sat;}

  this->left = left;
  this->right = right;
}




PIDConfig DriveController::fastVelConfig()
{
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 10;
  config.Kd = 2;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 610; //gives 127 pwm at 0.4 commandedspeed  ORIG:320
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::fastYawConfig() {
  PIDConfig config;

  config.Kp = 60;
  config.Ki = 15;
  config.Kd = 5;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/6;
  config.errorHistLength = 4;
  config.alwaysIntegral = false;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/3;
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

  config.Kp = 60;
  config.Ki = 10;
  config.Kd = 2;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 610; //gives 127 pwm at 0.4 commandedspeed  ORIG:320
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper/2;
  config.derivativeAlpha = 0.7;

  return config;

}

PIDConfig DriveController::constYawConfig() {
  PIDConfig config;

  config.Kp = 5;
  config.Ki = 5;
  config.Kd = 0;
  config.satUpper = 255;
  config.satLower = -255;
  config.antiWindup = config.satUpper/4;
  config.errorHistLength = 4;
  config.alwaysIntegral = true;
  config.resetOnSetpoint = true;
  config.feedForwardMultiplier = 0;
  config.integralDeadZone = 0.01;
  config.integralErrorHistoryLength = 10000;
  config.integralMax = config.satUpper;
  config.derivativeAlpha = 0.6;

  return config;

}
