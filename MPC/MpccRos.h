#ifndef __MPCC_ROS__
#define __MPCC_ROS__

#include "ros/ros.h"
#include "std_msgs/float64MultiArray.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

class MpccRos
{
private:
    // mpcc
    PathToJson jsonPath;
    Integrator integrator;
    Plotting plotter;
    MPC mpc;
    std::list<MPCReturn> log;

    // ros
    ros::Subscriber ekf_state_sub;
    ros::Subscriber ref_path_sub;

    ros::Publisher control_pub;

public:
    MpccRos(ros::NodeHandle &n, json JsonConfig);
    ~MpccRos();

private:
    void ekfStateCallback(const std_msgs::float64MultiArrayConstPtr& msg);

}

#endif // __MPCC_ROS__