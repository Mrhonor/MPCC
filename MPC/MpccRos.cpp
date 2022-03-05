#include "MPC/MpccRos.h"

MpccRos::MpccRos(json JsonConfig):
jsonPath(jsonConfig["model_path"],
jsonConfig["cost_path"],
jsonConfig["bounds_path"],
jsonConfig["track_path"],
jsonConfig["normalization_path"]),
integrator(jsonConfig["Ts"],jsonPath),
plotter(jsonConfig["Ts"],jsonPath),
mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths)

{

}