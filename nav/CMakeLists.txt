cmake_minimum_required(VERSION 3.10)

# --- Paths to ORB-SLAM3 build ---
set(ORB_SLAM3_ROOT "/tmp/ORB_SLAM3")
set(ORB_SLAM3_INCLUDE_DIRS
    ${ORB_SLAM3_ROOT}
    ${ORB_SLAM3_ROOT}/include
    ${ORB_SLAM3_ROOT}/include/CameraModels
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
)
set(ORB_SLAM3_LIBRARIES
    ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
)

# --- Nav library ---
add_library(nav
    orb_slam3_wrapper.cpp
    mbl.cpp
    vb_gps_integrator.cpp
    scene_understanding.cpp
    landmark_map.cpp
    navigator.cpp
)
target_include_directories(nav PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(nav PUBLIC
    ${ORB_SLAM3_LIBRARIES}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${EIGEN3_LIBS}
    boost_serialization
    crypto
)
target_compile_features(nav PUBLIC cxx_std_17)

