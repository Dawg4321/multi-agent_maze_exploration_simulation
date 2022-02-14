#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <list>

#include "GridGraph.h"

struct Coordinates{
    unsigned int x;
    unsigned int y;

    bool operator<(const Coordinates& a) const{
        return (x < a.x || y < a.y);
    }
};

class Robot{
    public:
        Robot(int x, int y);

        void scanCell(GridGraph* maze);
        bool move2Cell(int direction);
        bool pf_BFS(int x, int y);

        std::vector<Coordinates> getValidNeighbours(unsigned int x, unsigned  int y);

        void printRobotXMap();
        void printRobotYMap();
        void printRobotNodes();

    private:
        // cartesian cordinates
        unsigned int x_position; // x position within cells 
        unsigned int y_position; // y position within cells
        std::vector<Coordinates> planned_path; 
        
        GridGraph LocalMap; // local_map maintained by robot of areas explored
};



#endif