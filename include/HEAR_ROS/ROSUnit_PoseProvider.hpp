#ifndef ROSUNIT_POSEPROVIDER_HPP
#define ROSUNIT_POSEPROVIDER_HPP

#include <vector>

#include "ros/ros.h"
#ifdef PX4
    #include <mavros_msgs/VehicleAttitude.h>
    #include <mavros_msgs/VehicleAngularVelocity.h>
#endif
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <hear_msgs/set_float.h>

#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{

class ROSUnit_PoseProvider{
private:
    ros::NodeHandle nh_;
    ros::Subscriber opti_sub, vision_sub, px4_pos_sub, px4_vel_sub, px4_ori_sub, px4_ang_vel_sub, xsens_ori_sub, xsens_ang_vel_sub, xsens_free_acc_sub;
    ros::ServiceServer m_server;
    
    ExternalOutputPort<Vector3D<float>>* opti_pos_port;
    ExternalOutputPort<Vector3D<float>>* opti_vel_port;
    ExternalOutputPort<Vector3D<float>>* opti_ori_port;
    ExternalOutputPort<Vector3D<float>>* px4_pos_port;
    ExternalOutputPort<Vector3D<float>>* px4_vel_port;
    ExternalOutputPort<Vector3D<float>>* px4_ori_port;
    ExternalOutputPort<Vector3D<float>>* vision_pos_port;
    ExternalOutputPort<Vector3D<float>>* vision_vel_port;
    ExternalOutputPort<Vector3D<float>>* vision_ori_port;
    ExternalOutputPort<Vector3D<float>>* imu_ori_port;
    ExternalOutputPort<Vector3D<float>>* imu_acc_port;
    ExternalOutputPort<Vector3D<float>>* imu_angular_rt_port;
    ExternalOutputPort<Vector3D<float>>* px4_imu_angular_rt_port;
    void callback_opti_pose(const geometry_msgs::PoseStamped::ConstPtr& );
    void callback_vision_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );
    void callback_ori(const geometry_msgs::QuaternionStamped::ConstPtr& );
    void callback_free_acc(const geometry_msgs::Vector3Stamped::ConstPtr& );
    void callback_angular_vel(const geometry_msgs::Vector3Stamped::ConstPtr&);
    #ifdef PX4
    void callback_px4_pose(const geometry_msgs::PoseStamped::ConstPtr& );
    void callback_px4_vel(const geometry_msgs::TwistStamped::ConstPtr& );
    void callback_px4_ori(const mavros_msgs::VehicleAttitude::ConstPtr& );
    void callback_px4_angular_vel(const mavros_msgs::VehicleAngularVelocity::ConstPtr&);
    #endif
    bool srv_callback(hear_msgs::set_float::Request&, hear_msgs::set_float::Response&);
    tf2::Matrix3x3 rot_offset, rot_offset_vision, rot_offset_PX4_ori, rot_offset_PX4_yaw;
    tf2::Vector3 trans_offset, trans_offset_vision;

    tf2::Vector3 opti_pos, prev_pos, opti_vel, prev_diff, _hold;
    tf2::Vector3 vision_pos, prev_pos_vision, vision_vel, prev_diff_vision, _hold_vision;
    ros::Time prevT, prevT_vision;
    uint8_t first_read = 0;
    uint8_t first_read_vision = 0;
    const float PEAK_THRESH = 0.35;

public:
    void process(){}
    ROSUnit_PoseProvider(ros::NodeHandle& nh);
    ~ROSUnit_PoseProvider(){}
    std::vector<ExternalOutputPort<Vector3D<float>>*> registerOptiPose(std::string t_name);
    std::vector<ExternalOutputPort<Vector3D<float>>*> registerVisionPose(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerPX4Pose(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerPX4Vel(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerPX4ImuOri(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerPX4ImuAngularRate(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuOri(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuAngularRate(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuAcceleration(std::string t_name);
};

}

#endif