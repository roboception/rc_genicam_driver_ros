
set(SRC
    genicam_device_nodelet.cpp
    publishers/camera_info_publisher.cpp
    publishers/camera_param_publisher.cpp
    publishers/image_publisher.cpp
    publishers/disparity_publisher.cpp
    publishers/disparity_color_publisher.cpp
    publishers/depth_publisher.cpp
    publishers/confidence_publisher.cpp
    publishers/error_disparity_publisher.cpp
    publishers/error_depth_publisher.cpp
    publishers/points2_publisher.cpp
    publishers/genicam2ros_publisher.h)

# build and install the nodelet

link_directories(${RC_GENICAM_API_LIBRARY_DIRS})

add_library(rc_genicam_driver_nodelet ${SRC})

target_link_libraries(rc_genicam_driver_nodelet
  PUBLIC
    ${catkin_LIBRARIES}
  PRIVATE
    rc_genicam_api::rc_genicam_api
    ${RC_DYNAMICS_API_LIBRARIES})

add_executable(rc_genicam_driver genicam_device_node.cpp)
target_link_libraries(rc_genicam_driver ${catkin_LIBRARIES})

install(TARGETS rc_genicam_driver rc_genicam_driver_nodelet
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_dependencies(rc_genicam_driver_nodelet rc_genicam_driver_gencfg)
