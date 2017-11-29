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

    ClawController();

public:
    static ClawController* instance();

    void registerPublishers(ros::Publisher& fingerPublisher, ros::Publisher wristPublisher);

    bool wristDown();
    bool wristUp();
    bool fingerClose();
    bool fingerOpen();
};



#endif
