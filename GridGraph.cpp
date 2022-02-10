#include "GridGraph.h"

void printNodes(GridGraph* m){ // prints nodes of the referenced grid graph

    printf(" # |"); 

    for (int i = 0; i < 32; i++) // printing column numbers
        printf(" %-2d",i);

    printf("\n   "); // new line
    
    for (int i = 0; i < 32; i++) // printing divider from column numbers
        printf("---",i);

    for (int i = 0; i < 31; i++){
        printf("\n%3d|",i);
        for (int j = 0; j < 31; j++)
            printf(" %-2d",m->nodes[i][j]);
    }

    printf("\n");
}

void printXEdges(GridGraph* m){ // prints x_edge of the referenced grid graph

    printf(" # |"); 

    for (int i = 0; i < 32; i++) // printing column numbers
        printf(" %-2d",i);

    printf("\n   "); // new line
    
    for (int i = 0; i < 32; i++) // printing divider from column numbers
        printf("---",i);

    for (int i = 0; i < 32; i++){ // print x_edge matrix
        printf("\n%3d|",i);
        for (int j = 0; j < 32; j++)
            printf(" %-2d",static_cast<int>(m->x_edges[i][j]));
    }

    printf("\n");
}

void printYEdges(GridGraph* m){ // prints y_edge of the referenced grid graph

    printf(" # |"); 

    for (int i = 0; i < 32; i++) // printing column numbers
        printf(" %-2d",i);

    printf("\n   "); // new line
    
    for (int i = 0; i < 32; i++) // printing divider from column numbers
        printf("---",i);

    for (int i = 0; i < 32; i++){ // print x_edge matrix
        printf("\n%3d|",i);
        for (int j = 0; j < 32; j++)
            printf(" %-2d", static_cast<int>(m->y_edges[i][j]));
    }

    printf("\n");
}