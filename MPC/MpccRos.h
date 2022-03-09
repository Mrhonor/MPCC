#ifndef __MPCC_ROS__
#define __MPCC_ROS__

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace mpcc{
class MpccRos
{
private:
    // mpcc
    PathToJson jsonPath;
    Integrator integrator;
    Plotting plotter;
    MPC mpc;
    std::list<MPCReturn> log;

    bool isSetTrack;
    Track cur_track;

    // ros
    ros::Subscriber ekf_state_sub; // 状态向量订阅
    ros::Subscriber ref_path_sub;  // 参照路径订阅

    ros::Publisher control_pub;    // 控制向量输出

    
public:
    MpccRos(ros::NodeHandle &n, json JsonConfig);
    ~MpccRos(){}

private:
    // 状态回调函数
    void ekfStateCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

    // reference path 回调函数
    void refPathCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

};
}

#endif // __MPCC_ROS__