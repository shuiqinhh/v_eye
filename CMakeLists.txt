cmake_minimum_required(VERSION 3.16.3)

project(my_v_eye)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavcodec libswscale libavutil)
find_package(Pangolin REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)

target_link_libraries(tools ${GST_LIBS})

####################src###################

add_executable(record_video src/record_video.cpp tools/hik_video_recorder.cpp)
add_executable(run_slam
    src/run_slam.cpp
)


target_link_libraries(run_slam
    nav io tools
    ${ORB_SLAM3_LIBRARIES}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${EIGEN3_LIBS}
    fmt::fmt
    yaml-cpp
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost_serialization
    crypto
    usb-1.0
)
target_link_libraries(record_video ${OpenCV_LIBS} fmt::fmt tools io)

target_compile_features(run_slam PUBLIC cxx_std_17)

add_executable(test_mbl
    src/test_mbl.cpp
)
target_link_libraries(test_mbl
    nav io tools
    ${ORB_SLAM3_LIBRARIES}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${EIGEN3_LIBS}
    fmt::fmt
    yaml-cpp
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost_serialization
    crypto
    usb-1.0
)
target_compile_features(test_mbl PUBLIC cxx_std_17)

add_executable(test_vb_gps
    src/test_vb_gps.cpp
)
target_link_libraries(test_vb_gps
    nav io tools
    ${ORB_SLAM3_LIBRARIES}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${EIGEN3_LIBS}
    fmt::fmt
    yaml-cpp
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost_serialization
    crypto
    usb-1.0
)
target_compile_features(test_vb_gps PUBLIC cxx_std_17)

add_executable(test_scene
    src/test_scene.cpp
)
target_link_libraries(test_scene
    nav io tools
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    fmt::fmt
    yaml-cpp
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    usb-1.0
)
target_compile_features(test_scene PUBLIC cxx_std_17)

add_executable(test_full
    src/test_full.cpp
)
target_link_libraries(test_full
    nav io tools
    ${ORB_SLAM3_LIBRARIES}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${EIGEN3_LIBS}
    fmt::fmt
    yaml-cpp
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    boost_serialization
    crypto
    usb-1.0
)
target_compile_features(test_full PUBLIC cxx_std_17)

target_include_directories(record_video PRIVATE ${PROJECT_SOURCE_DIR}/io/hikrobot/include)


##################calibration################
# add_executable(capture calibration/capture.cpp)
add_executable(capture_withoutq calibrate/capture_withoutq.cpp)
add_executable(calibrate_camera calibrate/calibrate_camera.cpp)
add_executable(calibrate_device2world_handeye calibrate/calibrate_device2world_handeye.cpp)

target_link_libraries(capture_withoutq ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_device2world_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)

####################nav####################
add_subdirectory(nav)


