#include "Maze.h"

Maze::Maze(){ 
}

void Maze::generateInterconnectedMaze(int x_size, int y_size){
    maze_xsize = x_size; // setting maze x and y size
    maze_ysize = y_size;

    // generating an x_size by y_size sized grid of valid nodes
    for (int i = 0; i < maze_ysize; i++)
        for (int j = 0; j < maze_xsize; j++)
            MazeMap.nodes[i][j] = true;

    // generating valid connections in y direction movement
    // Map is fully interconnected thus only first and last row have '1' values 
    for (int i = 0; i < maze_ysize + 1; i++){
        MazeMap.y_edges[0][i] = true;
        MazeMap.y_edges[maze_ysize][i] = true;
    }

    for (int i = 0; i < maze_xsize + 1; i++){
        MazeMap.x_edges[i][0] = true;
        MazeMap.x_edges[i][maze_xsize] = true;
    }

    return;
}

void Maze::generate4x4SampleMaze(){
    maze_xsize = 4; // setting maze x and y size to 4x4
    maze_ysize = 4;

    char n[4][4] = { // marking nodes for maze
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                        {1, 1, 1, 1},
                    };

    for (int i = 0; i < sizeof(n)/sizeof(n[0]); i++) // passing nodes into graph struct
        for (int j = 0; j < sizeof(n[0])/sizeof(n[0][0]); j++)
            MazeMap.nodes[i][j] = n[i][j];
    

    char x[4][5] = { // marking x edges for maze
                        {1, 0, 1, 0, 1},
                        {1, 1, 0, 0, 1},
                        {1, 1, 0, 0, 1},
                        {1, 0, 1, 1, 1},
                    };

    for (int i = 0; i < sizeof(x)/sizeof(x[0]); i++) // passing x edges into graph struct
        for (int j = 0; j < sizeof(x[0])/sizeof(x[0][0]); j++)
            MazeMap.x_edges[i][j] = x[i][j];

    char y[5][4] = { // marking y edges for maze
                        {1, 1, 1, 1}, 
                        {0, 0, 1, 0},
                        {0, 0, 1, 1},
                        {0, 1, 0, 0},
                        {1, 1, 1, 1}
                    };

    for (int i = 0; i < sizeof(y)/sizeof(y[0]); i++) // passing y edges into graph struct
        for (int j = 0; j < sizeof(y[0])/sizeof(y[0][0]); j++)
            MazeMap.y_edges[i][j] = y[i][j];
    
    return;
}

void Maze::setMazeMap(GridGraph* m){
    MazeMap = *m;
}

GridGraph Maze::getMazeMap(){
    return (MazeMap);
}

GridGraph* Maze::getMazeMapPointer(){
    return (&MazeMap);
}