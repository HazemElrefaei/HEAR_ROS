// #include "HEAR_ROS/ROSUnit_EmptyClnt.hpp"

// namespace HEAR{

// ROSUnitEmptyClient::ROSUnitEmptyClient(ros::NodeHandle &nh, std::string t_name) : nh_(nh){
//     m_client = nh_.serviceClient<std_srvs::Empty>(t_name);
// }

// bool ROSUnitEmptyClient::process(){
//     std_srvs::Empty msg;    
//     return m_client.call(msg);
// }

// }