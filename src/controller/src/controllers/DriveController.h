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
      STATE_MACHINE_SKID_STEER,
    };

    StateMachineStates stateMachineState;
    float rotateOnlyAngleTolerance = 0.175;  //5 deg
    float finalRotationTolerance = 0.1;
    const float waypointTolerance = 0.15; //15 cm tolerance.

    float scaler = 0.5;
    float searchVelocity = 0.65; // meters/second  //0.65 MAX value
    float yawVelocity = 0.65;
   
    int leftRightMin = 50;

    ros::Publisher drivePublisher;
    geometry_msgs::Twist velocity;

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

    float left;
    float right;
    float linear;
    float angular;

    // for storing current drive command
    // We will use to see if drive changed before current was completed
    Point currentDrive;
    // For updating the current location of the robot
    // So taht we do not get from handler each time. Get it only once per tick for faster processing
    Point currentLocation;

    // Used to store the current distance eand dirrection
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


    public:
        static DriveController* instance();

        /**
         * @brief registerDrivePublisher - to register a drive publisher. Has to only be called once
         * @param drivePublisher - publisher connected to wheels
         */
        void registerDrivePublisher(ros::Publisher& drivePublisher);

        bool goToLocation(float x, float y);
        bool goToDistance(float distance, float direction);
        bool stop();
        void sendDriveCommand(double left, double right);
        void resetDriveController(float x, float y);

};


#endif
