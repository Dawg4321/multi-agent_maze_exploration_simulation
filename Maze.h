#ifndef MAZE_H
#define MAZE_H

#include <iostream>
#include "GridGraph.h"

class Maze
{
    public:
        Maze();

        void generateInterconnectedMaze(int x_size, int y_size);
        void generate4x4SampleMaze();

        GridGraph getMazeMap();
        GridGraph* getMazeMapPointer(); // returns pointer to MazeMap
                                        // be careful when using it as modifying this will modify private data member which is undesireable
                                        // using this over getMazeMap allows for performance improvements as large maze will not need to be copied
        void setMazeMap(GridGraph m);

    private:
        GridGraph MazeMap;
        unsigned int maze_xsize;
        unsigned int maze_ysize;

};

#endif