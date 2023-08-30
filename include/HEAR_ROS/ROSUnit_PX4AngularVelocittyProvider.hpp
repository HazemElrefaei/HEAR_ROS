#ifndef ROSUNIT_PX4ANGULARVELOCITYPROVIDER_HPP
#define ROSUNIT_PX4ANGULARVELOCITYPROVIDER_HPP

#include <vector>

#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>

#include <mavros_msgs/VehicleAngularVelocity.h>

#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{

class ROSUnit_PX4AngularVelocittyProvider{
private:
    ros::NodeHandle nh_;
    ros::Subscriber px4_ang_vel_sub;
    ros::ServiceServer m_server;
    
    ExternalOutputPort<Vector3D<float>>* imu_angular_rt_port;
    void callback_angular_vel(const mavros_msgs::VehicleAngularVelocity::ConstPtr&);

public:
    void process(){}
    ROSUnit_PX4AngularVelocittyProvider(ros::NodeHandle& nh);
    ~ROSUnit_PX4AngularVelocittyProvider(){}
    ExternalOutputPort<Vector3D<float>>* registerImuAngularRate(std::string t_name);
};

}

#endif