ament_add_gtest(utils_test utils_test.cpp)
target_link_libraries(utils_test dwb_core)

ament_add_gtest(publisher_test publisher_test.cpp TIMEOUT 45
  ENV ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
target_link_libraries(publisher_test dwb_core)
ament_target_dependencies(publisher_test ${dependencies})
