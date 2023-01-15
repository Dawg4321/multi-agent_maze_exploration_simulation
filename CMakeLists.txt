cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# setting project name
project(Multi-Agent_Maze_Simulator)

# including dependencies
include(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/CMakeLists.txt)

# adding executables
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(Multi-Agent_Maze_Simulator ${SRC_FILES})

# linking local libraries
target_include_directories(Multi-Agent_Maze_Simulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(Multi-Agent_Maze_Simulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Frontier_Exploration/Robot)
target_include_directories(Multi-Agent_Maze_Simulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Frontier_Exploration/RobotMaster)

# linking dependencies
target_link_libraries(Multi-Agent_Maze_Simulator fmt::fmt)
target_link_libraries(Multi-Agent_Maze_Simulator nlohmann_json::nlohmann_json)