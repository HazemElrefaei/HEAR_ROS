#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"

namespace HEAR{

ROSUnit_PoseProvider::ROSUnit_PoseProvider(ros::NodeHandle& nh): nh_(nh){
}


std::vector<ExternalOutputPort<Vector3D<float>>*> ROSUnit_PoseProvider::registerOptiPose(std::string t_name){
    m_server = nh_.advertiseService("set_height_offset", &ROSUnit_PoseProvider::srv_callback, this);
    rot_offset.setRPY(0.0, 0.0, 0.0);
    trans_offset.setZero();
    
    opti_pos_port = new ExternalOutputPort<Vector3D<float>>();
    opti_pos_port->write(Vector3D<float>(0,0,0));
    opti_vel_port = new ExternalOutputPort<Vector3D<float>>();
    opti_vel_port->write(Vector3D<float>(0,0,0));
    opti_ori_port = new ExternalOutputPort<Vector3D<float>>();
    opti_ori_port->write(Vector3D<float>(0,0,0));
    opti_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_opti_pose, this, ros::TransportHints().tcpNoDelay());
    return std::vector<ExternalOutputPort<Vector3D<float>>*>{opti_pos_port, opti_vel_port, opti_ori_port};
}

std::vector<ExternalOutputPort<Vector3D<float>>*> ROSUnit_PoseProvider::registerVisionPose(std::string t_name){
    rot_offset_vision.setRPY(0.0, 0.0, 0.0);
    trans_offset_vision.setZero();

    vision_pos_port = new ExternalOutputPort<Vector3D<float>>();
    vision_pos_port->write(Vector3D<float>(0,0,0));
    vision_vel_port = new ExternalOutputPort<Vector3D<float>>();
    vision_vel_port->write(Vector3D<float>(0,0,0));
    vision_ori_port = new ExternalOutputPort<Vector3D<float>>();
    vision_ori_port->write(Vector3D<float>(0,0,0));
    vision_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_vision_pose, this, ros::TransportHints().tcpNoDelay());
    return std::vector<ExternalOutputPort<Vector3D<float>>*>{vision_pos_port, vision_vel_port, vision_ori_port};
}

#ifdef PX4
ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerPX4Pose(std::string t_name){
    m_server = nh_.advertiseService("set_height_offset", &ROSUnit_PoseProvider::srv_callback, this);
    rot_offset.setRPY(0.0, 0.0, 0.0);
    trans_offset.setZero();
    
    px4_pos_port = new ExternalOutputPort<Vector3D<float>>();
    px4_pos_port->write(Vector3D<float>(0,0,0));
    // px4_vel_port = new ExternalOutputPort<Vector3D<float>>();
    // px4_vel_port->write(Vector3D<float>(0,0,0));
    // px4_ori_port = new ExternalOutputPort<Vector3D<float>>();
    // px4_ori_port->write(Vector3D<float>(0,0,0));
    px4_pos_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_px4_pose, this, ros::TransportHints().tcpNoDelay());
    return px4_pos_port;
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerPX4Vel(std::string t_name){
    px4_vel_port = new ExternalOutputPort<Vector3D<float>>();
    px4_vel_port->write(Vector3D<float>(0,0,0));

    px4_vel_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_px4_vel, this, ros::TransportHints().tcpNoDelay());
    return px4_vel_port;
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerPX4ImuOri(std::string t_name){
    rot_offset_PX4_ori.setRPY(M_PI, 0.0, 0.0); 

    px4_ori_port = new ExternalOutputPort<Vector3D<float>>();
    px4_ori_port->write(Vector3D<float>(0,0,0));
    px4_ori_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_px4_ori, this, ros::TransportHints().tcpNoDelay());
    return px4_ori_port;
}
ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerPX4ImuAngularRate(std::string t_name){
    px4_imu_angular_rt_port = new ExternalOutputPort<Vector3D<float>>();
    px4_imu_angular_rt_port->write(Vector3D<float>(0,0,0));
    px4_ang_vel_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_px4_angular_vel, this, ros::TransportHints().tcpNoDelay());
    return px4_imu_angular_rt_port;
}
#endif

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuOri(std::string t_name){
    imu_ori_port = new ExternalOutputPort<Vector3D<float>>();
    imu_ori_port->write(Vector3D<float>(0,0,0));
    xsens_ori_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_ori, this, ros::TransportHints().tcpNoDelay());
    return imu_ori_port;
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuAngularRate(std::string t_name){
    imu_angular_rt_port = new ExternalOutputPort<Vector3D<float>>();
    imu_angular_rt_port->write(Vector3D<float>(0,0,0));
    xsens_ang_vel_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_angular_vel, this, ros::TransportHints().tcpNoDelay());
    return imu_angular_rt_port;
}
ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuAcceleration(std::string t_name){
    imu_acc_port = new ExternalOutputPort<Vector3D<float>>();
    imu_acc_port->write(Vector3D<float>(0,0,0));
    xsens_free_acc_sub = nh_.subscribe(t_name, 10, &ROSUnit_PoseProvider::callback_free_acc, this, ros::TransportHints().tcpNoDelay());
    return imu_acc_port;
}

bool ROSUnit_PoseProvider::srv_callback(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res) {
    trans_offset.setZ(req.data);
    return true;
}

void ROSUnit_PoseProvider::callback_opti_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    
    tf2::Vector3 vel;
    auto pos = tf2::Vector3({msg->pose.position.x, msg->pose.position.y, msg->pose.position.z});
    auto calib_pos = rot_offset*pos - trans_offset;

    Vector3D<float> vec = {(float)calib_pos.x(), (float)calib_pos.y(), (float)calib_pos.z()};

    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w ));

    R_mat = rot_offset * R_mat * rot_offset.transpose();
    tf2Scalar yaw, pitch, roll;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec_ori = {(float)roll, (float)pitch, (float)yaw};

    // velocity calculation
    if(first_read == 0){
        first_read = 1;
        prevT = msg->header.stamp;
        prev_pos = pos;
        vel = tf2::Vector3(0, 0, 0);
        prev_diff = vel;
    }else{
        auto _dt = (msg->header.stamp - prevT).toSec();
        auto diff = (pos - prev_pos)/_dt;
        vel = diff;
        if(first_read == 1){
            first_read = 2;
            prev_diff = diff;
        }
        auto d_diff = diff - prev_diff;
        if(abs(d_diff.x()) > PEAK_THRESH || abs(d_diff.y()) > PEAK_THRESH || abs(d_diff.z()) > PEAK_THRESH){
            vel = _hold;
        }
        else{
            _hold = diff;
        }
        prev_diff = diff;
        prev_pos = pos;
        prevT = msg->header.stamp;
    }
    opti_vel = rot_offset*vel;
    ////////////////////////

    opti_pos_port->write(vec);
    opti_vel_port->write(Vector3D<float>(opti_vel.x(), opti_vel.y(), opti_vel.z()));
    opti_ori_port->write(vec_ori);
}

void ROSUnit_PoseProvider::callback_vision_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    
    tf2::Vector3 vel;
    auto pos = tf2::Vector3({msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z});
    auto calib_pos = rot_offset_vision*pos - trans_offset_vision;

    Vector3D<float> vec = {(float)calib_pos.x(), (float)calib_pos.y(), (float)calib_pos.z()};

    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w ));

    R_mat = rot_offset_vision * R_mat * rot_offset_vision.transpose();
    tf2Scalar yaw, pitch, roll;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec_ori = {(float)roll, (float)pitch, (float)yaw};

    // velocity calculation
    if(first_read_vision == 0){
        first_read_vision = 1;
        prevT_vision = msg->header.stamp;
        prev_pos_vision = pos;
        vel = tf2::Vector3(0, 0, 0);
        prev_diff_vision = vel;
    }else{
        auto _dt = (msg->header.stamp - prevT_vision).toSec();
        auto diff = (pos - prev_pos_vision)/_dt;
        vel = diff;
        if(first_read_vision == 1){
            first_read_vision = 2;
            prev_diff_vision = diff;
        }
        auto d_diff = diff - prev_diff;
        if(abs(d_diff.x()) > PEAK_THRESH || abs(d_diff.y()) > PEAK_THRESH || abs(d_diff.z()) > PEAK_THRESH){
            vel = _hold_vision;
        }
        else{
            _hold_vision = diff;
        }
        prev_diff_vision = diff;
        prev_pos_vision = pos;
        prevT_vision = msg->header.stamp;
    }
    vision_vel = rot_offset_vision*vel;
    ////////////////////////

    vision_pos_port->write(vec);
    vision_vel_port->write(Vector3D<float>(vision_vel.x(), vision_vel.y(), vision_vel.z()));
    vision_ori_port->write(vec_ori);
}



void ROSUnit_PoseProvider::callback_ori(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
    
    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w));

    tf2Scalar yaw, roll, pitch;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec = {(float)roll, (float)pitch, (float)yaw};
    imu_ori_port->write(vec);
}

void ROSUnit_PoseProvider::callback_angular_vel(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    Vector3D<float> vec = {(float)msg->vector.x, (float)msg->vector.y, (float)msg->vector.z};

    imu_angular_rt_port->write(vec);
}

#ifdef PX4
void ROSUnit_PoseProvider::callback_px4_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    
    auto pos = tf2::Vector3({msg->pose.position.x, msg->pose.position.y, msg->pose.position.z});
    auto calib_pos = rot_offset*pos - trans_offset;

    Vector3D<float> vec = {(float)calib_pos.x(), (float)calib_pos.y(), (float)calib_pos.z()};

    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w ));

    // R_mat = rot_offset * R_mat * rot_offset.transpose();
    tf2Scalar yaw, pitch, roll;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec_ori = {(float)roll, (float)pitch, (float)yaw};


    px4_pos_port->write(vec);
}
void ROSUnit_PoseProvider::callback_px4_vel(const geometry_msgs::TwistStamped::ConstPtr& msg){

    auto vel = tf2::Vector3({msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z});
    auto calib_vel = rot_offset*vel - trans_offset;

    Vector3D<float> vec = {(float)calib_vel.x(), (float)calib_vel.y(), (float)calib_vel.z()};


    px4_vel_port->write(vec);
}
void ROSUnit_PoseProvider::callback_px4_ori(const mavros_msgs::VehicleAttitude::ConstPtr& msg){
    
    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->q_x, msg->q_y, msg->q_z, msg->q_w ));

    R_mat = rot_offset_PX4_ori * R_mat * rot_offset_PX4_ori.transpose();
    tf2Scalar yaw, pitch, roll;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec_ori = {(float)roll, (float)pitch, (float)yaw};


    px4_ori_port->write(vec_ori);
}
void ROSUnit_PoseProvider::callback_px4_angular_vel(const mavros_msgs::VehicleAngularVelocity::ConstPtr& msg){
    Vector3D<float> vec = {(float)msg->angular_velocity_x, -(float)msg->angular_velocity_y, -(float)msg->angular_velocity_z};

    px4_imu_angular_rt_port->write(vec);
}
#endif

void ROSUnit_PoseProvider::callback_free_acc(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    Vector3D<float> vec = {(float)msg->vector.x, (float)msg->vector.y, (float)msg->vector.z};

    imu_acc_port->write(vec);
}

}