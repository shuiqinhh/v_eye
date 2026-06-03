cmake_minimum_required(VERSION 3.10)
project(V_Eye_Project)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找第三方库
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# 包含头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

# 收集源文件
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/*.h")

# 添加可执行文件
add_executable(V_Eye ${SOURCES} main.cpp)

# 链接库
target_link_libraries(V_Eye ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${EIGEN3_LIBRARIES})