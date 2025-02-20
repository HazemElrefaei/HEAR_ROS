// #ifndef ROSUNIT_MRFTSWITCHSRV_HPP
// #define ROSUNIT_MRFTSWITCHSRV_HPP

// #include <ros/ros.h>
// #include <hear_msgs/set_float.h>

// #include "HEAR_core/ExternalTrigger.hpp"

// #include <string>
// #include <vector>
// #include <utility>


// namespace HEAR{

// class ROSUnit_MRFTSwitchSrv {
// private:
//     ExternalTrigger<BaseMsg>* _pid_trig;
//     ExternalTrigger<BaseMsg>* _mrft_trig;
//     std::vector <std::pair<ExternalTrigger<BaseMsg>*, bool>> _switch_trig;
//     ros::ServiceServer m_server;
//     bool srvCallback(hear_msgs::set_float::Request& , hear_msgs::set_float::Response& );
// public:
//     ROSUnit_MRFTSwitchSrv(ros::NodeHandle& nh , const std::string& topic_name ){
//         m_server = nh.advertiseService(topic_name, &ROSUnit_MRFTSwitchSrv::srvCallback, this);
//         _pid_trig = new ExternalTrigger<BaseMsg>();
//         _mrft_trig = new ExternalTrigger<BaseMsg>();
//     }
//     ExternalTrigger<BaseMsg>* getMRFTTrig(){ return _mrft_trig;}
//     ExternalTrigger<BaseMsg>* getPIDTrig(){ return _pid_trig;}
//     ExternalTrigger<BaseMsg>* registerSwitchTrig(bool inverted_logic = false){
//         auto trig = new ExternalTrigger<BaseMsg>();
//         _switch_trig.push_back(std::make_pair(trig, inverted_logic));
//         return trig;
//     }

// };

// bool ROSUnit_MRFTSwitchSrv::srvCallback(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res){
//     float data = req.data;
//     if(data <= 0){
//         BoolMsg msg;
//         msg.data = false;
//         _mrft_trig->UpdateCallback((BaseMsg*)&msg);
//         msg.data = true;
//         _pid_trig->UpdateCallback((BaseMsg*)&msg);
//         SwitchMsg sw_msg;
//         for (const auto& trig : _switch_trig){
//             if(trig.second){
//                 sw_msg.sw_state = SWITCH_STATE::ON;
//             }
//             else{
//                 sw_msg.sw_state = SWITCH_STATE::OFF;
//             }
//             trig.first->UpdateCallback((BaseMsg*)&sw_msg);
//         }

//     }
//     else{
//         BoolMsg msg;
//         msg.data = true;
//         _mrft_trig->UpdateCallback((BaseMsg*)&msg);
//         msg.data = false;
//         _pid_trig->UpdateCallback((BaseMsg*)&msg);
//         SwitchMsg sw_msg;
//         for (const auto& trig : _switch_trig){
//             if(trig.second){
//                 sw_msg.sw_state = SWITCH_STATE::OFF;
//             }
//             else{
//                 sw_msg.sw_state = SWITCH_STATE::ON;
//             }
//             trig.first->UpdateCallback((BaseMsg*)&sw_msg);
//         }
//     }
//     return true;
// }

// }

// #endif
