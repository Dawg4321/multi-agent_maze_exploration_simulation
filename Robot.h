#ifndef ROBOT_H
#define ROBOT_H

#include "GridGraph.h"
#include <iostream>
#include <vector>

class Robot{
    public:
        Robot(int x, int y);

        void scanCell(GridGraph* maze);
        bool move2Cell(int direction);
        bool pf_FloodFill(int x, int y);

        void printRobotXMap();
        void printRobotYMap();
        void printRobotNodes();

    private:
        // cartesian cordinates
        unsigned int x_position; // x position within cells 
        unsigned int y_position; // y position within cells
        unsigned int planned_path[20]; 
        
        GridGraph local_map; // local_map maintained by robot of areas explored
};



#endif