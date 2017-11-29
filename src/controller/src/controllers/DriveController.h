#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include "PID.h"
#include "../handlers/Handlers.h"
#include "../Point.h"



class DriveController{
    static DriveController *s_instance; //static instance of class

    float rotateOnlyAngleTolerance;
    float finalRotationTolerance;
    float waypointTolerance;
    float searchVelocity;

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

    bool isInitThetaCalculated;
    float initTheta;

    //for timings
    int initTime;
    bool isInitTime;

    // for storing initial location
    Point initLocation;
    bool isInitLocation;
    static int spinCounter;

    DriveController();


    public:
        static DriveController* instance();

        /**
         * @brief registerDrivePublisher - to register a drive publisher. Has to only be called once
         * @param drivePublisher - publisher connected to wheels
         */
        void registerDrivePublisher(ros::Publisher& drivePublisher);

        bool goToLocation(float x, float y);
        bool spinInCircle(float spinVel, int spinTimes);
        bool goToDistance(float distance, float direction);
        bool stop();
        void sendDriveCommand(double left, double right);

};


#endif
