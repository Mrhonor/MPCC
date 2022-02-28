cmake_minimum_required(VERSION 3.7)
project(MPCC)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
  DEPENDS
    ${LIBS}
)

include_directories(.)
include_directories(External/blasfeo/lib/include)
include_directories(External/hpipm/lib/include)
include_directories(External/matplotlib)
include_directories(External/Eigen)
include_directories(External/Json/include)
include_directories(${catkin_INCLUDE_DIRS})


add_executable(MPCC
        main.cpp
        types.cpp
        types.h
        config.h
        Params/params.cpp
        Spline/cubic_spline.cpp
        Spline/arc_length_spline.cpp
        Interfaces/hpipm_interface.cpp
        Interfaces/solver_interface.cpp
        Constraints/constraints.cpp
        Constraints/bounds.cpp
        Cost/cost.cpp
        Model/model.cpp
        Model/integrator.cpp
        Tests/spline_test.cpp
        Tests/cost_test.cpp
        Tests/model_integrator_test.cpp
        Tests/constratins_test.cpp
        MPC/mpc.cpp
        Params/track.cpp
        Params/track.h
        Plotting/plotting.cpp
        Plotting/plotting.h)
add_dependencies(MPCC ${${PROJECT_NAME}_EXPORTTED_TARGETS}
${catkin_EXPORTED_TARGETS}
)

find_package(Python COMPONENTS Development)
target_include_directories(MPCC PRIVATE ${Python_INCLUDE_DIRS} ${catkin_LIBRARIES})
target_link_libraries(MPCC ${Python_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(MPCC ${CMAKE_SOURCE_DIR}/MPCC/External/hpipm/lib/lib/libhpipm.a ${CMAKE_SOURCE_DIR}/MPCC/External/blasfeo/lib/lib/libblasfeo.a m)
