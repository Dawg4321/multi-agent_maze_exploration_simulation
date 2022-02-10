#ifndef GRIDGRAPH_H
#define GRIDGRAPH_H

#include <iostream>
#include <vector>

#define INITIAL_NUM_ROWS 31 // number of rows preallocated in gridgraph grid
#define INITIAL_NUM_COLS 31 // number of collumns preallocated in gridgraph grid

typedef struct{
    char nodes[INITIAL_NUM_ROWS][INITIAL_NUM_COLS] = {0}; // Nodes represent each cell within the maze
                                                                  // 0 = invalid node
                                                                  // 1 = valid node

    // 32x32 bit matrix corresponding to y (east-west) edges of nodes in maze                                                                                         
    std::vector<std::vector<bool>> x_edges{INITIAL_NUM_ROWS + 1, std::vector<bool>(INITIAL_NUM_COLS + 1)}; // 32x32 bit matrix
    // 32x32 bit matrix corresponding to y (north-south) edges of nodes in maze
    std::vector<std::vector<bool>> y_edges{INITIAL_NUM_ROWS + 1, std::vector<bool>(INITIAL_NUM_COLS + 1)}; 
}GridGraph;

// print function declarations
void printXEdges(GridGraph* m);
void printYEdges(GridGraph* m);
void printNodes(GridGraph* m);

#endif