#ifndef MAZE_H
#define MAZE_H

#include <iostream>
#include <string>

#include "GridGraph.h"

class Maze
{
    public:
        Maze();

        void generateInterconnectedMaze(int x_size, int y_size); // function to generate a x * y sized maze with no walls
        void generate4x4SampleMaze();
        void generateCollisionTest();

        GridGraph getMazeMap(); // MazeMap getter
        GridGraph* getMazeMapPointer(); // returns pointer to MazeMap
                                        // be careful when using it as modifying this will modify private data member which is undesireable
                                        // using this over getMazeMap allows for performance improvements as large maze will not need to be copied
        void setMazeMap(GridGraph* m); // MazeMap setter

        bool printMaze();

    private:
        GridGraph MazeMap;
        unsigned int maze_xsize;
        unsigned int maze_ysize;

};

#endif