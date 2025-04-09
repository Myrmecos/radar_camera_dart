cmake_minimum_required(VERSION 3.10)

project(hikrobot_camera)

set(allowableBuildTypes Debug Release)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CUDA REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    cv_bridge
)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    /opt/MVS/include/
)

LINK_DIRECTORIES(
    /opt/MVS/lib/64/
)

add_executable(${PROJECT_NAME} src/hikrobot_camera.cpp)
add_executable(image_pub src/image_pub.cpp)

ament_target_dependencies(${PROJECT_NAME}
    ${dependencies}
)

ament_target_dependencies(image_pub
    ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    
    yaml-cpp

    MvCameraControl
    MVGigEVisionSDK
    MVRender
    MvUsb3vTL
    MediaProcess
)

target_link_libraries(image_pub
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    
    yaml-cpp

    MvCameraControl
    MVGigEVisionSDK
    MVRender
    MvUsb3vTL
    MediaProcess
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS image_pub
    DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()