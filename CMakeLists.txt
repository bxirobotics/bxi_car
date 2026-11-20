cmake_minimum_required(VERSION 3.16)
project(chassis LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Application, modules, configuration and package templates live under src/.
set(CHASSIS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Protocol internals stay separate; the two public entry files are communication/motor.
set(ISWV_PROTOCOL "${CHASSIS_SOURCE_DIR}/thirdparty/motor/iswv/protocol")
add_library(iswv_canopen STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/iswv/communication.cpp"
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/iswv/diagnostic_log.cpp"
  ${ISWV_PROTOCOL}/src/fake_transport.cpp
  ${ISWV_PROTOCOL}/src/pdo.cpp
  ${ISWV_PROTOCOL}/src/drive.cpp
  ${ISWV_PROTOCOL}/src/axis.cpp
  ${ISWV_PROTOCOL}/src/faults.cpp
  ${ISWV_PROTOCOL}/src/module.cpp
  ${ISWV_PROTOCOL}/src/steering_layout.cpp)
add_library(iswv::canopen ALIAS iswv_canopen)
set_target_properties(iswv_canopen PROPERTIES EXPORT_NAME canopen CXX_EXTENSIONS OFF)
target_compile_features(iswv_canopen PUBLIC cxx_std_17)
target_include_directories(iswv_canopen PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty>
  $<BUILD_INTERFACE:${ISWV_PROTOCOL}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(iswv_canopen PUBLIC Threads::Threads)
target_compile_options(iswv_canopen PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wconversion -Wshadow>)

add_library(chassis_bxi_transport STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/board.cpp"
  "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/can_timing.cpp")
target_include_directories(chassis_bxi_transport PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_bxi_transport PUBLIC iswv::canopen
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/lib/libbxi_pci_drv.a>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/lib/libbxi_pci_drv.a>)
# The retained vendor archive initializes CAN0..CAN6 at a fixed rate.
target_link_options(chassis_bxi_transport INTERFACE
  "-Wl,--wrap=XCanFd_SetBaudRatePrescaler" "-Wl,--wrap=XCanFd_SetBitTiming")

add_library(chassis_bxi_motor STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/bxi/communication.cpp"
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/bxi/motor.cpp")
target_link_libraries(chassis_bxi_motor PUBLIC iswv::canopen)
add_library(chassis_iswv_motor STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/motor/iswv/motor.cpp")
target_link_libraries(chassis_iswv_motor PUBLIC iswv::canopen)

set(SOEM "${CHASSIS_SOURCE_DIR}/thirdparty/motor/yiyou/vendor/soem")
file(GLOB SOEM_SOURCES CONFIGURE_DEPENDS ${SOEM}/soem/*.c)
add_library(chassis_soem STATIC ${SOEM_SOURCES}
  ${SOEM}/osal/linux/osal.c ${SOEM}/oshw/linux/oshw.c ${SOEM}/oshw/linux/nicdrv.c)
target_include_directories(chassis_soem PRIVATE
  ${SOEM}/soem ${SOEM}/osal ${SOEM}/osal/linux ${SOEM}/oshw/linux)
target_link_libraries(chassis_soem PUBLIC Threads::Threads rt)
add_library(chassis_ethercat STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/yiyou/communication.cpp"
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/yiyou/cycle_timing.cpp")
target_include_directories(chassis_ethercat PRIVATE
  ${SOEM}/soem ${SOEM}/osal ${SOEM}/osal/linux ${SOEM}/oshw/linux)
target_link_libraries(chassis_ethercat PUBLIC iswv::canopen chassis_soem)
add_library(chassis_yiyou_motor STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/yiyou/motor.cpp"
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/yiyou/position_memory.cpp")
target_compile_definitions(chassis_yiyou_motor PRIVATE
  CHASSIS_DEFAULT_POSITION_MEMORY_FILE="${CMAKE_CURRENT_SOURCE_DIR}/state/yiyou_motor4_position.yaml")
target_link_libraries(chassis_yiyou_motor PUBLIC chassis_ethercat yaml-cpp Threads::Threads)

add_library(chassis_arm_kinematics STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/arm/kinematics.cpp")
target_include_directories(chassis_arm_kinematics PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty> $<INSTALL_INTERFACE:include>)
add_library(chassis_arm_control STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/arm/control.cpp"
  "${CHASSIS_SOURCE_DIR}/thirdparty/arm/torque_log.cpp")
target_link_libraries(chassis_arm_control PUBLIC
  chassis_arm_kinematics chassis_yiyou_motor chassis_bxi_transport yaml-cpp)
add_library(chassis_controller STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/chassis/control.cpp")
target_link_libraries(chassis_controller PUBLIC
  chassis_iswv_motor chassis_bxi_transport yaml-cpp)

add_library(chassis_can_relay STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/relay/control.cpp")
target_link_libraries(chassis_can_relay PUBLIC iswv::canopen)

# ROS belongs to the application layer; all hardware/domain modules also work without ROS.
add_library(chassis_remote_control STATIC
  "${CHASSIS_SOURCE_DIR}/example/remote_control.cpp"
  "${CHASSIS_SOURCE_DIR}/example/shared_control_server.cpp"
  "${CHASSIS_SOURCE_DIR}/example/joystick_link_monitor.cpp")
target_include_directories(chassis_remote_control PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(chassis_remote_control PUBLIC rclcpp rcl_interfaces geometry_msgs std_msgs sensor_msgs)
target_link_libraries(chassis_remote_control PUBLIC yaml-cpp Threads::Threads)
# Keep existing ROS executable names; there is only one maintained main.cpp.
foreach(application chassis key_control robot_control chassis_only chassis_only_remote)
  add_executable(${application} "${CHASSIS_SOURCE_DIR}/example/main.cpp")
  target_compile_definitions(${application} PRIVATE
    CHASSIS_SOURCE_CONFIG_DIR="${CHASSIS_SOURCE_DIR}/config"
    CHASSIS_TORQUE_REPORT_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/tools/arm_torque_report.py")
  target_link_libraries(${application} PRIVATE
    chassis_controller chassis_arm_control chassis_remote_control chassis_can_relay)
endforeach()
target_compile_definitions(chassis PRIVATE CHASSIS_CONTROL_MODE="chassis")
target_compile_definitions(key_control PRIVATE CHASSIS_CONTROL_MODE="combined")
target_compile_definitions(robot_control PRIVATE CHASSIS_CONTROL_MODE="combined")
target_compile_definitions(chassis_only PRIVATE CHASSIS_CONTROL_MODE="chassis_only")
target_compile_definitions(chassis_only_remote PRIVATE CHASSIS_CONTROL_MODE="chassis_only_remote")

# Diagnostic backends are reached through test/debug.py.
foreach(tool arm_yiyou_test arm_yiyou_set_id arm_yiyou_control)
  add_executable(${tool} test/tools/${tool}.cpp)
  target_link_libraries(${tool} PRIVATE chassis_yiyou_motor chassis_bxi_transport)
endforeach()
add_executable(chassis_main test/tools/chassis_main.cpp)
target_link_libraries(chassis_main PRIVATE chassis_controller chassis_arm_control)
add_executable(can_relay_setup test/tools/can_relay_setup.cpp)
target_include_directories(can_relay_setup PRIVATE "${CHASSIS_SOURCE_DIR}")
target_link_libraries(can_relay_setup PRIVATE chassis_bxi_transport)
add_executable(motor_power_hold test/tools/motor_power_hold.cpp)
target_link_libraries(motor_power_hold PRIVATE chassis_bxi_transport)
add_executable(bxi_motor_offline test/tools/bxi_motor_offline.cpp)
target_link_libraries(bxi_motor_offline PRIVATE chassis_bxi_motor)
add_executable(motor_console test/tools/motor_console.cpp)
target_link_libraries(motor_console PRIVATE
  chassis_bxi_motor chassis_iswv_motor chassis_bxi_transport)
add_executable(arm_solver test/tools/arm_solver.cpp)
target_link_libraries(arm_solver PRIVATE chassis_arm_control)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_imu_heading_guard test/unit/test_imu_heading_guard.cpp)
  target_include_directories(test_imu_heading_guard PRIVATE ${CHASSIS_SOURCE_DIR})
  ament_target_dependencies(test_imu_heading_guard sensor_msgs)
  add_test(NAME imu_heading_guard COMMAND test_imu_heading_guard)
  set_tests_properties(imu_heading_guard PROPERTIES TIMEOUT 10)
  add_test(NAME hipnuc_imu COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_hipnuc_imu.py)
  set_tests_properties(hipnuc_imu PROPERTIES TIMEOUT 20)
  # Link vendor stubs directly: this test must never access a real BXI board.
  add_executable(test_bxi_runtime test/unit/test_bxi_runtime.cpp
    test/unit/bxi_can_timing_stub.cpp
    "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/board.cpp"
    "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/can_timing.cpp")
  add_executable(test_can_timing test/unit/test_can_timing.cpp
    "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/can_timing.cpp")
  target_include_directories(test_can_timing PRIVATE "${CHASSIS_SOURCE_DIR}/thirdparty")
  target_link_libraries(test_can_timing PRIVATE Threads::Threads)
  add_test(NAME can_timing COMMAND test_can_timing)
  set_tests_properties(can_timing PROPERTIES TIMEOUT 10)
  add_executable(test_can_relay test/unit/test_can_relay.cpp)
  target_link_libraries(test_can_relay PRIVATE chassis_can_relay)
  add_test(NAME can_relay COMMAND test_can_relay)
  set_tests_properties(can_relay PROPERTIES TIMEOUT 10)
  target_include_directories(test_bxi_runtime PRIVATE
    "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/include")
  target_link_libraries(test_bxi_runtime PRIVATE iswv::canopen)
  add_test(NAME bxi_runtime COMMAND test_bxi_runtime)
  set_tests_properties(bxi_runtime PROPERTIES TIMEOUT 10)
  add_executable(test_steering_config test/unit/test_steering_config.cpp)
  target_link_libraries(test_steering_config PRIVATE iswv::canopen yaml-cpp)
  add_test(NAME steering_config COMMAND test_steering_config
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/fixtures/steering.yaml
    ${CHASSIS_SOURCE_DIR}/config/steering.yaml)
  add_executable(test_regression test/unit/regression.cpp)
  target_link_libraries(test_regression PRIVATE chassis_bxi_motor chassis_arm_kinematics)
  add_test(NAME regression COMMAND test_regression)
  add_executable(test_motor_layers test/unit/test_motor_layers.cpp)
  target_link_libraries(test_motor_layers PRIVATE
    chassis_bxi_motor chassis_iswv_motor chassis_yiyou_motor)
  add_test(NAME motor_layers COMMAND test_motor_layers)
  set_tests_properties(motor_layers PROPERTIES TIMEOUT 30)
  add_executable(test_ethercat_timing test/unit/test_ethercat_timing.cpp)
  target_link_libraries(test_ethercat_timing PRIVATE chassis_ethercat)
  add_test(NAME ethercat_timing COMMAND test_ethercat_timing)
  set_tests_properties(ethercat_timing PROPERTIES TIMEOUT 15)
  add_executable(test_ethercat_health test/unit/test_ethercat_health.cpp)
  target_link_libraries(test_ethercat_health PRIVATE chassis_ethercat)
  add_test(NAME ethercat_health COMMAND test_ethercat_health)
  set_tests_properties(ethercat_health PROPERTIES TIMEOUT 15)
  add_executable(test_ethercat_interface test/unit/test_ethercat_interface.cpp)
  target_link_libraries(test_ethercat_interface PRIVATE chassis_ethercat)
  target_link_options(test_ethercat_interface PRIVATE
    "-Wl,--wrap=if_nametoindex" "-Wl,--wrap=getifaddrs" "-Wl,--wrap=freeifaddrs"
    "-Wl,--wrap=ec_init" "-Wl,--wrap=socket")
  add_test(NAME ethercat_interface COMMAND test_ethercat_interface)
  set_tests_properties(ethercat_interface PROPERTIES TIMEOUT 5)
  add_executable(test_position_memory test/unit/test_position_memory.cpp)
  target_link_libraries(test_position_memory PRIVATE chassis_yiyou_motor)
  add_test(NAME position_memory COMMAND test_position_memory)
  set_tests_properties(position_memory PROPERTIES TIMEOUT 30)
  add_executable(test_yiyou_control_memory test/unit/test_yiyou_control_memory.cpp)
  target_link_libraries(test_yiyou_control_memory PRIVATE chassis_yiyou_motor chassis_bxi_transport)
  add_test(NAME yiyou_control_memory COMMAND test_yiyou_control_memory)
  set_tests_properties(yiyou_control_memory PROPERTIES TIMEOUT 30)
  foreach(torque_test arm_torque_feedback arm_torque_log)
    add_executable(test_${torque_test} test/unit/test_${torque_test}.cpp)
    target_link_libraries(test_${torque_test} PRIVATE chassis_arm_control)
    add_test(NAME ${torque_test} COMMAND test_${torque_test})
    set_tests_properties(${torque_test} PROPERTIES TIMEOUT 20)
  endforeach()
  set_tests_properties(arm_torque_feedback PROPERTIES TIMEOUT 45)
  add_test(NAME arm_torque_report COMMAND /usr/bin/python3 -I -S
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_arm_torque_report.py)
  set_tests_properties(arm_torque_report PROPERTIES TIMEOUT 60)
  add_test(NAME gemini_imu COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_gemini_imu.py)
  set_tests_properties(gemini_imu PROPERTIES TIMEOUT 5)
  foreach(diagnostic_test can_diagnostic_log can_diagnostics)
    add_executable(test_${diagnostic_test} test/unit/test_${diagnostic_test}.cpp)
    target_link_libraries(test_${diagnostic_test} PRIVATE iswv::canopen)
    add_test(NAME ${diagnostic_test} COMMAND test_${diagnostic_test})
    set_tests_properties(${diagnostic_test} PROPERTIES TIMEOUT 20)
  endforeach()
  add_executable(test_calibration_feedback test/unit/test_calibration_feedback.cpp)
  target_link_libraries(test_calibration_feedback PRIVATE iswv::canopen)
  add_test(NAME calibration_feedback COMMAND test_calibration_feedback)
  set_tests_properties(calibration_feedback PROPERTIES TIMEOUT 20)
  add_executable(test_steering_calibration test/unit/test_steering_calibration.cpp)
  target_link_libraries(test_steering_calibration PRIVATE chassis_controller)
  add_test(NAME steering_calibration COMMAND test_steering_calibration)
  set_tests_properties(steering_calibration PROPERTIES TIMEOUT 20)
  add_test(NAME debug_menu COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_debug.py)
  set_tests_properties(debug_menu PROPERTIES TIMEOUT 30
    ENVIRONMENT "CHASSIS_TEST_BIN_DIR=${CMAKE_CURRENT_BINARY_DIR};PYTHONDONTWRITEBYTECODE=1")
  add_test(NAME startup_scripts COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_start_scripts.py)
  set_tests_properties(startup_scripts PROPERTIES TIMEOUT 30
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  add_test(NAME app_deployment COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_app_deployment.py)
  set_tests_properties(app_deployment PROPERTIES TIMEOUT 45)
  add_executable(test_app_interface test/unit/test_app_interface.cpp)
  target_link_libraries(test_app_interface PRIVATE chassis_remote_control chassis_arm_kinematics)
  add_test(NAME app_interface COMMAND test_app_interface)
  set_tests_properties(app_interface PROPERTIES TIMEOUT 15
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=194;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  # Vision remains a standalone optional Python feature; motor builds do not need its SDK.
  execute_process(COMMAND ${Python3_EXECUTABLE} -B -c "import cv2, numpy"
    RESULT_VARIABLE CHASSIS_VISION_IMPORT_RESULT OUTPUT_QUIET ERROR_QUIET)
  if(CHASSIS_VISION_IMPORT_RESULT EQUAL 0)
    foreach(vision_test qr_midpoint gemini_camera qr_midpoint_cli arm_tracking arm_approach
        docking_travel docking_verification arm_tracking_cli arm_tracking_ros)
      add_test(NAME ${vision_test} COMMAND ${Python3_EXECUTABLE} -B
        ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_${vision_test}.py)
      set_tests_properties(${vision_test} PROPERTIES TIMEOUT 30
        ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
    endforeach()
  else()
    message(STATUS "Vision tests unavailable: selected Python needs OpenCV and NumPy")
  endif()
  add_test(NAME ros_contract COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_ros_contract.py
    --bin-dir ${CMAKE_CURRENT_BINARY_DIR} --config ${CHASSIS_SOURCE_DIR}/config/steering.yaml)
  set_tests_properties(ros_contract PROPERTIES TIMEOUT 45
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  add_executable(test_remote_disconnect test/unit/test_remote_disconnect.cpp)
  target_link_libraries(test_remote_disconnect PRIVATE chassis_remote_control)
  add_test(NAME remote_disconnect COMMAND test_remote_disconnect)
  set_tests_properties(remote_disconnect PROPERTIES TIMEOUT 45
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=197;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  add_executable(test_arm_position_remote test/unit/test_arm_position_remote.cpp)
  target_link_libraries(test_arm_position_remote PRIVATE chassis_arm_control chassis_remote_control)
  add_test(NAME arm_position_remote COMMAND test_arm_position_remote)
  set_tests_properties(arm_position_remote PROPERTIES TIMEOUT 20
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=196;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  add_executable(test_arm_visual_tracking test/unit/test_arm_visual_tracking.cpp)
  target_link_libraries(test_arm_visual_tracking PRIVATE chassis_arm_control chassis_remote_control)
  add_test(NAME arm_visual_tracking COMMAND test_arm_visual_tracking)
  set_tests_properties(arm_visual_tracking PROPERTIES TIMEOUT 60
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=195;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  add_executable(test_velocity_command_watchdog test/unit/test_velocity_command_watchdog.cpp)
  target_include_directories(test_velocity_command_watchdog PRIVATE ${CHASSIS_SOURCE_DIR})
  add_test(NAME velocity_command_watchdog COMMAND test_velocity_command_watchdog)
  add_executable(test_shared_control test/unit/test_shared_control.cpp)
  target_link_libraries(test_shared_control PRIVATE chassis_remote_control)
  add_test(NAME shared_control COMMAND test_shared_control)
  set_tests_properties(shared_control PROPERTIES TIMEOUT 20)
  add_executable(test_shared_command_gate test/unit/test_shared_command_gate.cpp)
  target_include_directories(test_shared_command_gate PRIVATE ${CHASSIS_SOURCE_DIR})
  target_link_libraries(test_shared_command_gate PRIVATE yaml-cpp)
  add_test(NAME shared_command_gate COMMAND test_shared_command_gate)
  add_test(NAME shared_control_client COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_shared_control_client.py)
  set_tests_properties(shared_control_client PROPERTIES TIMEOUT 20)
  add_test(NAME shared_daemon COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_shared_daemon.py
    --binary $<TARGET_FILE:chassis_only_remote>)
  set_tests_properties(shared_daemon PROPERTIES TIMEOUT 20)
  # Vendor symbols are supplied by the stub object, so the static vendor
  # archive is never extracted. No CAN device or power hardware is accessed.
  add_executable(test_shared_recycle_daemon "${CHASSIS_SOURCE_DIR}/example/main.cpp"
    test/unit/shared_daemon_vendor_stub.cpp test/unit/bxi_can_timing_stub.cpp)
  target_compile_definitions(test_shared_recycle_daemon PRIVATE
    CHASSIS_CONTROL_MODE="chassis_only_remote"
    CHASSIS_SOURCE_CONFIG_DIR="${CHASSIS_SOURCE_DIR}/config"
    CHASSIS_TORQUE_REPORT_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/tools/arm_torque_report.py"
    CHASSIS_HARDWARE_LOCK_FILE="${CMAKE_CURRENT_BINARY_DIR}/shared-recycle-hardware.lock")
  target_link_libraries(test_shared_recycle_daemon PRIVATE
    chassis_controller chassis_arm_control chassis_remote_control chassis_can_relay)
  add_test(NAME shared_daemon_recycle COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_shared_daemon_recycle.py
    --binary $<TARGET_FILE:test_shared_recycle_daemon>)
  set_tests_properties(shared_daemon_recycle PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)
  add_executable(test_joystick_link_monitor test/unit/test_joystick_link_monitor.cpp)
  target_link_libraries(test_joystick_link_monitor PRIVATE chassis_remote_control)
  add_test(NAME joystick_link_monitor COMMAND test_joystick_link_monitor)
  set_tests_properties(joystick_link_monitor PROPERTIES TIMEOUT 15)
  foreach(application chassis key_control robot_control chassis_only chassis_only_remote
      arm_yiyou_control arm_yiyou_test arm_yiyou_set_id chassis_main motor_power_hold motor_console arm_solver can_relay_setup)
    add_test(NAME ${application}_help COMMAND ${application} --help)
    set_tests_properties(${application}_help PROPERTIES TIMEOUT 5)
  endforeach()
  add_test(NAME bxi_motor_offline COMMAND bxi_motor_offline)
endif()

install(TARGETS chassis key_control robot_control chassis_only chassis_only_remote
  arm_yiyou_test arm_yiyou_set_id arm_yiyou_control chassis_main motor_power_hold motor_console arm_solver can_relay_setup
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS test/debug.py test/tools/yiyou_control.py test/tools/can3_control.py
  tools/arm_torque_report.py tools/gemini_imu.py tools/hipnuc_imu.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS "${CHASSIS_SOURCE_DIR}/example/qr_midpoint.py"
  "${CHASSIS_SOURCE_DIR}/example/arm_tracking.py"
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY "${CHASSIS_SOURCE_DIR}/thirdparty/vision"
  DESTINATION lib/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
install(FILES requirements-vision.txt DESTINATION share/${PROJECT_NAME})
install(TARGETS chassis_bxi_transport chassis_bxi_motor chassis_iswv_motor
  chassis_soem chassis_ethercat chassis_yiyou_motor chassis_arm_kinematics
  chassis_arm_control chassis_controller chassis_remote_control chassis_can_relay
  EXPORT chassisControlTargets ARCHIVE DESTINATION lib)
install(DIRECTORY "${CHASSIS_SOURCE_DIR}/thirdparty/" DESTINATION include FILES_MATCHING
  PATTERN "*.h" PATTERN "*.hpp" PATTERN "vendor" EXCLUDE PATTERN "protocol" EXCLUDE)
install(DIRECTORY ${ISWV_PROTOCOL}/include/ DESTINATION include)
install(DIRECTORY "${CHASSIS_SOURCE_DIR}/example/" DESTINATION include/example FILES_MATCHING
  PATTERN "*.h" PATTERN "*.hpp")
install(FILES "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/include/bxi_pci_drv.h" DESTINATION include)
install(FILES "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/lib/libbxi_pci_drv.a" DESTINATION lib)

include(CMakePackageConfigHelpers)
install(TARGETS iswv_canopen EXPORT iswv_canopenTargets ARCHIVE DESTINATION lib)
install(EXPORT iswv_canopenTargets NAMESPACE iswv:: DESTINATION lib/cmake/iswv_canopen)
configure_package_config_file("${CHASSIS_SOURCE_DIR}/cmake/iswv_canopenConfig.cmake.in"
  ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfig.cmake INSTALL_DESTINATION lib/cmake/iswv_canopen)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfigVersion.cmake
  VERSION 0.1.0 COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfigVersion.cmake DESTINATION lib/cmake/iswv_canopen)
ament_export_include_directories(include)
ament_export_dependencies(iswv_canopen yaml-cpp rclcpp rcl_interfaces geometry_msgs std_msgs sensor_msgs)
ament_export_targets(chassisControlTargets HAS_LIBRARY_TARGET)
install(DIRECTORY "${CHASSIS_SOURCE_DIR}/config/" DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_app_control.sh scripts/shared_control.py scripts/postinstall.sh scripts/postinstall.py
  DESTINATION scripts)
install(FILES ${SOEM}/LICENSE ${SOEM}/ORIGIN.md ${SOEM}/LOCAL_PATCHES.md
  DESTINATION share/${PROJECT_NAME}/vendor/soem)
ament_package()
