# Multi-agent maze exploration simulator

This is a C++ simulator allows for a variety of multi-agent exploration systems to be simulated within a maze environment. The goal of this is to be able to evalute high-level exploration control schemes without considering low-level challenges of robotic systems such as localisation. This was done by  giving robots three main functionalities and placing it into a class - 1. move within the maze environment, 2. scan for walls within the maze environment, 3 - P2P communication with the centralised supervisor. During simulations, robot objects are instantiated and placed into seperate threads. These threads are synchronised into turns ensuring each robot has the opportunity to perform actions simultaneously. Each action is assigned an associated turn cost to simulate a real-world environment (i.e. robot moving requires more time than communicating with the supervisor). Data from simulations can be exported in json format for later analysis. Additionally, gifs of simulations can be created for easy viewing. Portions of this project was completed during my disseration at TU Dublin.

<p align="center">
  <img src="https://github.com/Dawg4321/multi-agent_maze_exploration_simulation/blob/master/examples/10x10_Greedy/maze_exploration.gif"/>
</p>

## Getting Started
The C++ simulator is designed for use with ```Linux``` due to the use of ```POSIX Threads```. Additionally, ```C++20``` and at minimum ```CMake v3.10``` are required. Before building, git submodules must be cloned into the local repository using the following command in the project's main directory:
```
git submodule update --init --recursive
```
It is suggested that a seperate build directory is created to store the various CMake files:
```
mkdir build && cd build
```
Now the required CMake files can be generated into ```build```:
```
cmake ..
```
The CMake files are now setup properly. To build the project, use the follwing in ```build```:
```
make
```
To run the simulator, use the following in the build directory:
```
./Multi-Agent_Maze_Simulator
```

