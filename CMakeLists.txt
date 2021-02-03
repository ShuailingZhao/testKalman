cmake_minimum_required(VERSION 2.8)
project(testKalman)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_PREFIX_PATH "/usr/local/protobuf")

set(PROJECT_ROOT ${PROJECT_SOURCE_DIR})

set("OpenCV_DIR" "/home/zhaosl/dataDisk/zhaosl/local_install/opencv3.3.0/share/OpenCV/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
MESSAGE("OpenCV vesion: ${OpenCV_VERSION}")


include_directories(${PROJECT_ROOT}/include)


file(GLOB testKalman_src ${PROJECT_ROOT}/src/testKalman.cpp ${PROJECT_ROOT}/src/positionKalmanFilter.cpp)
add_executable(testKalman ${testKalman_src})
target_link_libraries(testKalman ${OpenCV_LIBS})


file(GLOB testKalman2_src ${PROJECT_ROOT}/src/testKalman2.cpp ${PROJECT_ROOT}/src/positionKalmanFilter2.cpp)
add_executable(testKalman2 ${testKalman2_src})
target_link_libraries(testKalman2 ${OpenCV_LIBS})

file(GLOB testKalman3_src ${PROJECT_ROOT}/src/testKalman3.cpp ${PROJECT_ROOT}/src/positionKalmanFilter2.cpp)
add_executable(testKalman3 ${testKalman3_src})
target_link_libraries(testKalman3 ${OpenCV_LIBS})
