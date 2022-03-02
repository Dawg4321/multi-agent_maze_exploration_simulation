#ifndef GRIDGRAPH_H
#define GRIDGRAPH_H

#include <iostream>
#include <vector>

struct GridGraph{
    std::vector<std::vector<char>> nodes; // Nodes represent each cell within the maze
                                          // values for usage as Maze
                                          // 0 = node not apart of maze
                                          // 1 = node apart of maze
                                          // values for usage as Map
                                          // 0 = invalid node
                                          // 1 = explored valid node
                                          // 2 =  unexplored valid node
                                                     
    std::vector<std::vector<bool>> x_edges; // bit matrix corresponding to y (east-west) edges of nodes in maze 
    
    std::vector<std::vector<bool>> y_edges; // bit matrix corresponding to y (north-south) edges of nodes in maze

    GridGraph(){ //  creating empty GridGraph
                 // need to allocate various vectors manually
    }

    GridGraph(int x, int y){ // x = x length of graph, y = y length of graph
        nodes.resize(y,std::vector<char>(x, 0)); // allocating vectors to empty matrix of specified size 
        x_edges.resize(y+1,std::vector<bool>(x+1, false));
        y_edges.resize(y+1,std::vector<bool>(x+1, false));
    }
};

// print function declarations
void printXEdges(GridGraph* m);
void printYEdges(GridGraph* m);
void printNodes(GridGraph* m);

#endif