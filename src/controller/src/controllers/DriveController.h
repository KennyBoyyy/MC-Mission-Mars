#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include "PID.h"
#include "../handlers/Handlers.h"
#include "../Point.h"
#include <thread>


class DriveController{
    static DriveController *s_instance; //static instance of class

    // state machine states
    enum StateMachineStates {
      STATE_MACHINE_ROTATE = 0,
      FINAL_ROTATE,
      STATE_MACHINE_SKID_STEER
    };

    StateMachineStates stateMachineState;
    float rotateOnlyAngleTolerance = 0.262;
    float finalRotationTolerance = 0.0349;
    const float waypointTolerance = 0.15; //15 cm tolerance.

    float scaler = 0.5;
    float searchVelocity = 0.65; // meters/second  //0.65 MAX value
    float yawVelocity = 0.65;
   
    // The initial left min and right min values for the robot
    // Used when the robot never ran the calibration 
    double leftMin = 50;
    double rightMin = 50;

    ros::Publisher drivePublisher;
    geometry_msgs::Twist velocity;

    float linear;
    float angular;

    bool isDistanceTurnedInit = false;
    float prevDistanceTurned;
    float distanceTurned;
    float initDirection;
  
    //Max PWM is 255
    //abridge currently limits MAX to 120 to prevent over-current draw
    float left; //left wheels PWM value
    float right; //right wheel PWM value

    // for storing current drive command
    // We will use to see if drive changed before current was completed
    Point currentDrive;
    // For updating the current location of the robot
    // So that we do not get from handler each time. Get it only once per tick for faster processing
    Point currentLocation;

    // Used to store the current distance and direction
    // We will use to see if drive changed before current was completed
    float direction;
    float distance;

    DriveController(){
        stateMachineState = STATE_MACHINE_ROTATE;
        fastVelPID.SetConfiguration(fastVelConfig());
        fastYawPID.SetConfiguration(fastYawConfig());
        
        slowVelPID.SetConfiguration(slowVelConfig());
        slowYawPID.SetConfiguration(slowYawConfig());

        constVelPID.SetConfiguration(constVelConfig());
        constYawPID.SetConfiguration(constYawConfig());

        left = 0;
        right = 0;
        linear = 0;
        angular = 0;

        direction = 0;
        distance = 0;
    }

    //===============PID==============================//
    PID fastVelPID;
    PID fastYawPID;

    PID slowVelPID;
    PID slowYawPID;

    PID constVelPID;
    PID constYawPID;

    PIDConfig fastVelConfig();
    PIDConfig fastYawConfig();
    PIDConfig slowVelConfig();
    PIDConfig slowYawConfig();
    PIDConfig constVelConfig();
    PIDConfig constYawConfig();

    void fastPID(float errorVel, float errorYaw , float setPointVel, float setPointYaw);
    void slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw);
    void constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw);
    //================================================//


    public:
        static DriveController* instance();

        /**
         * @brief registerDrivePublisher - to register a drive publisher. Has to only be called once
         * @param drivePublisher - publisher connected to wheels
         */
        void registerDrivePublisher(ros::Publisher& drivePublisher);

        // Got to X and Y point 
        bool goToLocation(float x, float y);

        // Go a certain distance in a certain direction
        bool goToDistance(float distance, float direction);

        // Just turn to face that direction
        bool turnToTheta(float theta);

        // Drive straight a certain distance
        bool driveStraight(float distance);

        // Stop rover
        bool stop();

        // Reset controller values
        void resetDriveController(float x, float y);

        // send drive command to the publisher 
        void sendDriveCommand(double left, double right);

        // Turn right with speed
        void turnRight(double speed){sendDriveCommand(speed, -speed);}

        // Turn left with speed
        void turnLeft(double speed){sendDriveCommand(-speed, speed);}

        // Set the left and right wheel min value 
        void setLeftRightMin(double leftMin, double rightMin);

        // Get the left wheel min 
        double getLeftMin(){return leftMin;}

        // get right wheel min
        double getRightMin(){return rightMin;}



};


#endif
