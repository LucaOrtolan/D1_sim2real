cmake_minimum_required(VERSION 3.8)
project(d1_sim2real)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(UNITREE_SDK2_DIR /home/master26/unitree_sdk2)
set(UNITREE_DDS_LIB_DIR ${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64)

include_directories(
    # d1_bridge_node.cpp uses: #include "msg/ArmString_.hpp"
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    # msg/*.cpp use: #include "ArmString_.hpp"
    ${CMAKE_CURRENT_SOURCE_DIR}/src/msg
    ${UNITREE_SDK2_DIR}/include
    /usr/local/include/ddscxx
    /usr/local/include/iceoryx/v2.0.2
)

add_executable(d1_bridge_node
    src/msg/ArmString_.cpp
    src/msg/PubServoInfo_.cpp
)

ament_target_dependencies(d1_bridge_node rclcpp sensor_msgs)

target_link_libraries(d1_bridge_node
    ${UNITREE_SDK2_DIR}/lib/x86_64/libunitree_sdk2.a
    ${UNITREE_DDS_LIB_DIR}/libddsc.so
    ${UNITREE_DDS_LIB_DIR}/libddscxx.so
    rt
    pthread
)
set_target_properties(d1_bridge_node PROPERTIES
    INSTALL_RPATH ${UNITREE_DDS_LIB_DIR}
    BUILD_WITH_INSTALL_RPATH TRUE
)
target_link_options(d1_bridge_node PRIVATE -Wl,--disable-new-dtags)

add_executable(arm_zero_control
    src/msg/ArmString_.cpp
)

target_link_libraries(arm_zero_control
    ${UNITREE_SDK2_DIR}/lib/x86_64/libunitree_sdk2.a
    ${UNITREE_DDS_LIB_DIR}/libddsc.so
    ${UNITREE_DDS_LIB_DIR}/libddscxx.so
    rt
    pthread
)
set_target_properties(arm_zero_control PROPERTIES
    INSTALL_RPATH ${UNITREE_DDS_LIB_DIR}
    BUILD_WITH_INSTALL_RPATH TRUE
)
target_link_options(arm_zero_control PRIVATE -Wl,--disable-new-dtags)

install(TARGETS d1_bridge_node arm_zero_control
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    src/run_task.py
    src/d1_wrapper.py
    src/policy_controller.py
    src/config_loader.py
    src/zero_position.py
    src/rest_position.py
    src/d1_bridge_node.py
    src/move_joint.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY policy/
    DESTINATION share/${PROJECT_NAME}/policy
)

ament_package()