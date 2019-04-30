project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

find_package( Eigen3 REQUIRED )
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
include_directories(src src/planner/include)



set(sources src/main.cpp
            src/planner/src/trajectory/Generator.cpp
            src/planner/src/trajectory/helper.cpp

            src/planner/src/struct/WayPoint.cpp
            src/planner/src/struct/Trajectory.cpp
            src/planner/src/struct/Ego.cpp
            src/planner/src/struct/Map.cpp
            src/planner/src/struct/Vehicle.cpp

            src/planner/src/predictor/Predictor.cpp

            src/planner/src/state/Planner.cpp
            src/planner/src/state/StStop.cpp
            src/planner/src/state/StWaitForRun.cpp
            src/planner/src/state/StRun.cpp
            src/planner/src/state/StDrive.cpp)

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
target_link_libraries(path_planning ${Boost_LIBRARIES})
