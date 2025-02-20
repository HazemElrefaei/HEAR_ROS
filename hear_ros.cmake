set(HEAR_ROS_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
set(HEAR_ROS_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

file(GLOB HEAR_ROS_SRCs ${HEAR_ROS_SOURCE_DIR}/*.cpp)

if(PX4_DEPS)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        hear_msgs
        mavros_msgs
        std_srvs
        geometry_msgs
        tf2_geometry_msgs
        tf2
        tf2_ros
    )
else()
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        hear_msgs
        std_srvs
        geometry_msgs
        tf2_geometry_msgs
        tf2
        tf2_ros
    )
endif()

set(HEAR_ROS_LIBS ${catkin_LIBRARIES})