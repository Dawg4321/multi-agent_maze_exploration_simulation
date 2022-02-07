#include "Maze.h"

Maze::Maze(){ 
}

void Maze::generateInterconnectedMaze(int x_size, int y_size){
    maze_xsize = x_size; // setting maze x and y size
    maze_ysize = y_size;

    // generating an x_size by y_size sized grid of valid nodes
    for (int i = 0; i < maze_ysize; i++)
        for (int j = 0; j < maze_xsize; j++)
            MazeMap.nodes[i][j] = 1;

    // generating valid connections in y direction movement
    // Map is fully interconnected thus only first and last row have '1' values 
    for (int i = 0; i < maze_ysize + 1; i++){
        MazeMap.y_edges[0][i] = 1;
        MazeMap.y_edges[maze_ysize + 1][i] = 1;
    }

    for (int i = 0; i < maze_xsize + 1; i++){
        MazeMap.x_edges[i][0] = 1;
        MazeMap.x_edges[i][maze_xsize + 1] = 1;
    }

    return;
}

void Maze::generate4x4SampleMaze(){
    maze_xsize = 4; // setting maze x and y size to 4x4
    maze_ysize = 4;

    unsigned int n[31][31] = { // marking nodes for maze
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                    };

    for (int i = 0; i < 31; i++) // passing nodes into graph struct
        for (int j = 0; j < 31; j++)
            MazeMap.nodes[i][j] = n[i][j];
    

    unsigned int x[32][32] = { // marking x edges for maze
                        {1, 0, 1, 0, 1},
                        {1, 1, 0, 0, 1},
                        {1, 1, 0, 0, 1},
                        {1, 0, 1, 1, 1},
                    };

    for (int i = 0; i < 32; i++) // passing x edges into graph struct
        for (int j = 0; j < 32; j++)
            MazeMap.x_edges[i][j] = x[i][j];

    unsigned int y[32][32] = { // marking y edges for maze
                        {1, 1, 1, 1},
                        {0, 0, 1, 0},
                        {0, 0, 1, 1},
                        {0, 1, 0, 0},
                        {1, 1, 1, 1},
                    };

    for (int i = 0; i < 32; i++) // passing y edges into graph struct
        for (int j = 0; j < 32; j++)
            MazeMap.y_edges[i][j] = y[i][j];
    
    return;
}

void Maze::setMazeMap(GridGraph m){
    MazeMap = m;
}

GridGraph Maze::getMazeMap(){
    return (MazeMap);
}

GridGraph* Maze::getMazeMapPointer(){
    return (&MazeMap);
}