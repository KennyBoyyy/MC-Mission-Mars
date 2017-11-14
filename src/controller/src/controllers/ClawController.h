#ifndef CLAWCONTROLLER_H
#define CLAWCONTROLLER_H

#include "ros/ros.h"
#include <std_msgs/Float32.h>

class ClawController
{
    static ClawController* s_instance;

    std_msgs::Float32 fingerOpenAngle;
    std_msgs::Float32 fingerCloseAngle;
    std_msgs::Float32 wristDownAngle;
    std_msgs::Float32 wristUpAngle;

    ros::Publisher fingerPublisher;
    ros::Publisher wristPublisher;

    ClawController() {
        fingerOpenAngle.data =  M_PI_2;
        fingerCloseAngle.data = 0.0;
        wristDownAngle.data = M_PI_2;
        wristUpAngle.data = 0.0;
    }

public:
    static ClawController* instance(){
        if(!s_instance)
            s_instance = new ClawController;
        return s_instance;
    }

    void registerPublishers(ros::Publisher& fingerPublisher, ros::Publisher wristPublisher){
        this->fingerPublisher = fingerPublisher;
        this->wristPublisher = wristPublisher;
    }

    bool wristDown();
    bool wristUp();
    bool fingerClose();
    bool fingerOpen();
};



#endif
