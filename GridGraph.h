#ifndef GRIDGRAPH_H
#define GRIDGRAPH_H

#include <iostream>

typedef struct{
    unsigned int nodes[31][31] = {0}; // Nodes represent each cell within the maze
                                      // 0 = invalid node
                                      // 1 = valid node                            
    unsigned int x_edges[32][32] = {0}; // 32x32 bit matrix
    unsigned int y_edges[32][32] = {0}; // 32x32 bit matrix corresponding to y (up-down) edges of nodes in maze
}GridGraph;

void printXEdges(GridGraph* m);
void printYEdges(GridGraph* m);
void printNodes(GridGraph* m);

#endif