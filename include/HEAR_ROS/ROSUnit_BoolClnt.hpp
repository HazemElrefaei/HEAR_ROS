#ifndef ROSUNIT_BOOLCLNT
#define ROSUNIT_BOOLCLNT

#include <ros/ros.h>
#include <hear_msgs/set_bool.h>
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_ROS/ROSUnit_Client.hpp"
#include <string>

namespace HEAR{

class ROSUnitBoolClient: public ROSUnit_Client<bool>{

public:
    ros::NodeHandle nh_;
    ROSUnitBoolClient(ros::NodeHandle& nh, std::string);
    bool process();
};

}

#endif