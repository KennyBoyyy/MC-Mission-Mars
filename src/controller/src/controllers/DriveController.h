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

    float rotateOnlyAngleTolerance = 0.15;
    float finalRotationTolerance = 0.15;
    const float waypointTolerance = M_PI_2; //15 cm tolerance.
    float searchVelocity = 0.5; // meters/second

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

    bool isInitThetaCalculated = false;
    float initTheta = 0;

    //for timings
    int initTime;
    bool isInitTime = false;

    // for storing initial location
    Point initLocation;
    bool isInitLocation = false;
    static int spinCounter;

    DriveController(){
        fastVelPID.SetConfiguration(fastVelConfig());
        fastYawPID.SetConfiguration(fastYawConfig());
        
        slowVelPID.SetConfiguration(slowVelConfig());
        slowYawPID.SetConfiguration(slowYawConfig());

        constVelPID.SetConfiguration(constVelConfig());
        constYawPID.SetConfiguration(constYawConfig());
    }

    void sendDriveCommand(double left, double right){
        velocity.linear.x = left;
        velocity.angular.z = right;

        // publish the drive commands
        drivePublisher.publish(velocity);
    }

    public:
        static DriveController* instance(){
            if (!s_instance)
              s_instance = new DriveController;
            return s_instance;
        }

        /**
         * @brief registerDrivePublisher - to register a drive publisher. Has to only be called once
         * @param drivePublisher - publisher connected to wheels
         */
        void registerDrivePublisher(ros::Publisher& drivePublisher){
            this->drivePublisher = drivePublisher;
        }

        bool goToLocation(float x, float y);
        bool spinInCircle(float spinVel, int spinTimes);
        bool goToDistance(float distance, float direction);
        bool stop();

};


#endif
