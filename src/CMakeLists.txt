find_package(PCL REQUIRED common io surface features)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_cnoid_simple_controller(AW_wheel_controller wheel_controller.cpp)
add_cnoid_simple_controller(kinect_data_recorder kinect_data_recorder.cpp)
add_cnoid_simple_controller(vlp_data_recorder vlp_data_recorder.cpp)


target_link_libraries (kinect_data_recorder ${PCL_COMMON_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} pcl_io)
target_link_libraries (vlp_data_recorder ${PCL_COMMON_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} pcl_io)
