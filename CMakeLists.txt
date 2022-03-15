cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the project name
project(Multi-Agent_Maze_Simulation)

# add the executable
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(Multi-Agent_Maze_Simulation ${SRC_FILES})
target_include_directories(Multi-Agent_Maze_Simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(Multi-Agent_Maze_Simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Robot)
target_include_directories(Multi-Agent_Maze_Simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/RobotMaster)