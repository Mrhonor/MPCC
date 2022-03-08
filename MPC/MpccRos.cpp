#include "MPC/MpccRos.h"

MpccRos::MpccRos(ros::NodeHandle &n, json JsonConfig):
jsonPath(jsonConfig["model_path"],
jsonConfig["cost_path"],
jsonConfig["bounds_path"],
jsonConfig["track_path"],
jsonConfig["normalization_path"]),
integrator(jsonConfig["Ts"],jsonPath),
plotter(jsonConfig["Ts"],jsonPath),
mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths)
{
    ekf_state_sub = n.subscribe("/EKF/State", 10, &MpccRos::ekfStateCallback, this);

    Track track = Track(jsonPath.track_path);
    TrackPos track_xy = track.getTrack();


    mpc.setTrack(track_xy.X,track_xy.Y);
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    State x0 = {track_xy.X(0),track_xy.Y(0),phi_0,jsonConfig["v0"],0,0,0,0.5,0,jsonConfig["v0"]};
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {
        MPCReturn mpc_sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0,mpc_sol.u0,jsonConfig["Ts"]);
        log.push_back(mpc_sol);
    }
    // plotter.plotRun(log,track_xy);
    plotter.plotSim(log,track_xy);

    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;
}

void MpccRos::ekfStateCallback(const std_msgs::float64MultiArrayConstPtr& msg){
    State x0;
    x0.X = msg->pose.pose.position.x;
    x0.Y = msg->pose.pose.position.y;
    Eigen::Quaterniod q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    x0.phi = q.matrix().eulerAngle(2,1,0)[2];
    x0.vx = msg->twist.twist.linear.x;
    x0.vy = msg->twist.twist.linear.y;
    x0.r = msg->twist.twist.angular.z;


}