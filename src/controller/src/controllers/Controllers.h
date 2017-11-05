#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include "PID.h"
#include "../handlers/Handlers.h"




class DriveController{
    static DriveController *s_instance; //static instance of class

    float rotateOnlyAngleTolerance = 0.3;
    float finalRotationTolerance = 0.15;
    const float waypointTolerance = 0.15; //15 cm tolerance.
    float searchVelocity = 0.5; // meters/second

    ros::Publisher drivePublisher;
    geometry_msgs::Twist velocity;

    PID fastVelPID;
    PID fastYawPID;

    PIDConfig fastVelConfig();
    PIDConfig fastYawConfig();

    void fastPID(float errorVel, float errorYaw , float setPointVel, float setPointYaw);

    float left;
    float right;

    DriveController(){
        fastVelPID.SetConfiguration(fastVelConfig());
        fastYawPID.SetConfiguration(fastYawConfig());
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

};


#endif
