#include "MPC/MpccRos.h"
#include <vector>
namespace mpcc{
MpccRos::MpccRos(ros::NodeHandle &n, json jsonConfig):
jsonPath{jsonConfig["model_path"],
jsonConfig["cost_path"],
jsonConfig["bounds_path"],
jsonConfig["track_path"],
jsonConfig["normalization_path"]},
integrator(jsonConfig["Ts"],jsonPath),
plotter(jsonConfig["Ts"],jsonPath),
mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],jsonPath)
{
    isSetTrack = false;

    ekf_state_sub = n.subscribe("/EKF/State", 10, &MpccRos::ekfStateCallback, this);
    ref_path_sub = n.subscribe("/RefPath", 10, &MpccRos::refPathCallback, this);

    control_pub = n.advertise<std_msgs::Float64MultiArray>("/MPCC/Control", 10);
    


}

void MpccRos::ekfStateCallback(const std_msgs::Float64MultiArrayConstPtr& msg){
    State x0;
    x0.X = msg->data[0];
    x0.Y = msg->data[1];
    // Eigen::Quaterniod q(msg->data[2], msg->data[3],
    //                     msg->data[4], msg->data[4]);
    // x0.phi = q.matrix().eulerAngle(2,1,0)[2];
    x0.phi = msg->data[2];
    x0.vx = msg->data[3];
    x0.vy = msg->data[4];
    x0.r = msg->data[5];
    x0.D = msg->data[6];
    x0.delta = msg->data[7];
    x0.vs = msg->data[8];

    // 临时使用，通知仿真结束
    double TempSimuEnd = msg->data[9];

    if(isSetTrack){
        MPCReturn mpc_sol = mpc.runMPC(x0);
        log.push_back(mpc_sol);
        std_msgs::Float64MultiArray control_msg;
        control_msg.data.push_back(mpc_sol.u0.dD);
        control_msg.data.push_back(mpc_sol.u0.dDelta);
        control_msg.data.push_back(mpc_sol.u0.dVs);
        if(TempSimuEnd < 2999){
            control_pub.publish(control_msg);
        }
        else{
            TrackPos track_xy = cur_track.getTrack();
            plotter.plotSim(log,track_xy);
            isSetTrack = false;
        }
        
    }
}


void MpccRos::refPathCallback(const std_msgs::Float64MultiArrayConstPtr& msg){

    if(msg->layout.dim[0].label == "X" && msg->layout.dim[1].label == "Y"
    && msg->layout.dim[2].label == "Xin" && msg->layout.dim[3].label == "Yin"
    && msg->layout.dim[4].label == "Xout" && msg->layout.dim[5].label == "Yout"){
        int x_size = msg->layout.dim[0].size, y_size = msg->layout.dim[1].size,
        xin_size = msg->layout.dim[2].size, yin_size = msg->layout.dim[3].size,
        xout_size = msg->layout.dim[4].size, yout_size = msg->layout.dim[5].size;
        std::vector<double> X,Y,Xin,Yin,Xout,Yout;
        int index=0;
        for(; index < x_size; index++){
            X.push_back(msg->data[index]);
            
        }
        for(; index-x_size < y_size; index++){
            Y.push_back(msg->data[index]);
        }
        for(; index-x_size-y_size < xin_size; index++){
            Xin.push_back(msg->data[index]);
        }
        for(; index-x_size-y_size-xin_size < yin_size; index++){
            Yin.push_back(msg->data[index]);
        }
        for(; index-x_size-y_size-xin_size-yin_size < xout_size; index++){
            Xout.push_back(msg->data[index]);
        }
        for(; index-x_size-y_size-xin_size-yin_size-xout_size < yout_size; index++){
            Yout.push_back(msg->data[index]);
        }
        cur_track.X = Eigen::Map<Eigen::VectorXd>(X.data(), X.size());
        cur_track.Y = Eigen::Map<Eigen::VectorXd>(Y.data(), Y.size());
        cur_track.X_inner = Eigen::Map<Eigen::VectorXd>(Xin.data(), Xin.size());
        cur_track.Y_inner = Eigen::Map<Eigen::VectorXd>(Yin.data(), Yin.size());
        cur_track.X_outer = Eigen::Map<Eigen::VectorXd>(Xout.data(), Xout.size());
        cur_track.Y_outer = Eigen::Map<Eigen::VectorXd>(Yout.data(), Yout.size());
        TrackPos track_xy = cur_track.getTrack();
        mpc.setTrack(track_xy.X,track_xy.Y);
        isSetTrack = true;
    }
    else{
        ROS_ERROR("Reference Path dones't accord with the protocol!");
        return;
    }
    
}
}