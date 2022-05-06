cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# setting project name
project(Multi-Agent_Maze_Simulation)

# adding executables
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(Multi-Agent_Maze_Simulation ${SRC_FILES})

# linking local libraries
target_include_directories(Multi-Agent_Maze_Simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(Multi-Agent_Maze_Simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Robot)
target_include_directories(Multi-Agent_Maze_Simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/RobotMaster)

find_package(fmt)
target_link_libraries(Multi-Agent_Maze_Simulation fmt::fmt)