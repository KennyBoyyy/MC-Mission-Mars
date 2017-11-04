#include "Behaviors.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
//=============================================================================================================//
//==============================================SimpleBehavior=================================================//
SimpleBehavior::SimpleBehavior(ros::Publisher& test){
    this->test = test;
}

bool SimpleBehavior::tick(){
    std_msgs::Float32 f;
    f.data = 1;
    test.publish(f);
}
