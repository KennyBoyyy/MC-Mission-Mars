#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "PID.h"




class DriveController{
    static DriveController *s_instance; //static instance of class
    ros::Publisher drivePublisher;
    geometry_msgs::Twist velocity;

    PIDConfig fastVelConfig();
    PIDConfig fastYawConfig();
    PIDConfig slowVelConfig();
    PIDConfig slowYawConfig();
    PIDConfig constVelConfig();
    PIDConfig constYawConfig();

    PID fastVelPID;
    PID fastYawPID;

    PID slowVelPID;
    PID slowYawPID;

    PID constVelPID;
    PID constYawPID;


    void fastPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw);
    void slowPID(float errorVel,float errorYaw, float setPointVel, float setPointYaw);
    void constPID(float erroVel,float constAngularError, float setPointVel, float setPointYaw);
    void ProcessData();

    void sendDriveCommand(double left, double right)
    {
      velocity.linear.x = left,
          velocity.angular.z = right;

      // publish the drive commands
      drivePublisher.publish(velocity);
    }

    //private by default
    DriveController(){
        fastVelPID.SetConfiguration(fastVelConfig());
        fastYawPID.SetConfiguration(fastYawConfig());

        slowVelPID.SetConfiguration(slowVelConfig());
        slowYawPID.SetConfiguration(slowYawConfig());

        constVelPID.SetConfiguration(constVelConfig());
        constYawPID.SetConfiguration(constYawConfig());
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


        void driveToPoint(float x, float y, float theta);

        void driveWithVelocity(float linear, float angular);

};


#endif
