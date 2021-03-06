#ifndef MAZE_H
#define MAZE_H

#include <iostream>
#include <string>
#include <random>
#include <stdexcept>

#include "GridGraph.h"
#include "Coordinates.h"

class Maze
{
    public:
        void generate4x4SampleMaze(); // generates the sample 4x4 maze
        void generate8x8SampleMaze(); // generates the sample 8x8 maze
        void generateRandomNxNMaze(unsigned int x_size, unsigned int y_size); // generates a random NxN maze using Aldous-Broder Maze generation Algorithm
        
        GridGraph getMazeMap(); // MazeMap getter
        GridGraph* getMazeMapPointer(); // returns pointer to MazeMap
                                        // be careful when using it as modifying this will modify private data member which is undesireable
                                        // using this over getMazeMap allows for performance improvements as large maze will not need to be copied

        unsigned int getMazeXSize(); // gets x size of maze
        unsigned int getMazeYSize(); // gets x size of maze

        bool printMaze();

    private:
        GridGraph MazeMap;
        unsigned int maze_xsize;
        unsigned int maze_ysize;

};

#endif