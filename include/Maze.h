#ifndef MAZE_H
#define MAZE_H

#include <iostream>
#include <string>

#include "GridGraph.h"

class Maze
{
    public:
        Maze();
        ~Maze();

        void generateInterconnectedMaze(unsigned int x_size, unsigned int y_size); // function to generate a x * y sized maze with no walls
        void generate4x4SampleMaze();
        void generate8x8SampleMaze();
        void generateCollisionTest();
        
        GridGraph getMazeMap(); // MazeMap getter
        GridGraph* getMazeMapPointer(); // returns pointer to MazeMap
                                        // be careful when using it as modifying this will modify private data member which is undesireable
                                        // using this over getMazeMap allows for performance improvements as large maze will not need to be copied

        unsigned int getMazeXSize(); // gets x size of maze
        unsigned int getMazeYSize(); // gets x size of maze

        bool printMaze();

    private:
        GridGraph* MazeMap;
        unsigned int maze_xsize;
        unsigned int maze_ysize;

};

#endif