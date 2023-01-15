# Multi-agent maze exploration simulator

This is a C++ simulator allows for a variety of multi-agent exploration systems to be simulated within a maze environment. The goal of this is to be able to evalute high-level exploration control schemes without considering low-level challenges of robotic systems such as localisation. Data from simulationss can be exported in json format for analysis purposes. Additionally, gifs of simulations can be created for easy viewing. Portions of this project was completed for my disseration at TU Dublin.

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

## Running Simulations

## Analysing Data

## Acknowledgements
