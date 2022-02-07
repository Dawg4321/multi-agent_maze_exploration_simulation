#include "GridGraph.h"

void printXEdges(GridGraph* m){
    for (int i = 0; i < 32; i++){
        std::cout << "\n" << i << ": ";
        for (int j = 0; j < 32; j++)
            std::cout << m->x_edges[i][j];
    }
}

void printYEdges(GridGraph* m){
    for (int i = 0; i < 32; i++){
        std::cout << "\n" << i << ": ";
        for (int j = 0; j < 32; j++)
            std::cout << m->y_edges[i][j];
    }
}

void printNodes(GridGraph* m){
    for (int i = 0; i < 31; i++){
        std::cout << "\n" << i << ": ";
        for (int j = 0; j < 31; j++)
            std::cout << m->nodes[i][j];
    }
}