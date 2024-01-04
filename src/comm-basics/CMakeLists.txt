cmake_minimum_required(VERSION 3.8)
project(comm-basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # 添加sensor_msgs的依赖项
find_package(OpenCV REQUIRED)  # 添加对OpenCV的依赖项

include_directories(${sensor_msgs_INCLUDE_DIRS})  # 添加sensor_msgs的包含目录
include_directories(${OpenCV_INCLUDE_DIRS})  # 添加OpenCV的包含目录

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
target_link_libraries(talker ${OpenCV_LIBRARIES} ${sensor_msgs_LIBRARIES})  # 链接OpenCV库以及sensor_msgs库

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
target_link_libraries(listener ${OpenCV_LIBRARIES} ${sensor_msgs_LIBRARIES})  # 链接OpenCV库以及sensor_msgs库


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
