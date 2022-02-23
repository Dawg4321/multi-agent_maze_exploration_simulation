#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <string>

#include "GridGraph.h"
#include "Coordinates.h"
#include "RequestHandler.h"

class Robot{
    public:
        Robot(int x, int y, RequestHandler* r);

        void scanCell(GridGraph* maze);

        bool move2Cell(int direction);
        bool move2Cell(Coordinates* destination);

        bool pf_BFS(int x, int y);

        bool BFS_pf2NearestUnknownCell(std::vector<Coordinates>* ret_vector);

        void startUp(GridGraph* maze);
        void soloExplore(GridGraph* maze);

        std::vector<Coordinates> getValidNeighbours(unsigned int x, unsigned  int y);

        bool printRobotMaze();
        void printRobotXMap();
        void printRobotYMap();
        void printRobotNodes();

    private:
        unsigned int x_position; // x position within cells 
        unsigned int y_position; // y position within cells

        int number_of_unexplored; // number of unexplored cells encountered

        unsigned int id; // robot id assigned by controller;

        std::vector<Coordinates> planned_path; 
        
        GridGraph LocalMap; // local_map maintained by robot of areas explored

        RequestHandler* Message_Handler; // pointer to request handler shared by all Robot objects

        unsigned int maze_xsize; // size of maze
        unsigned int maze_ysize; // this is not used by exploration algorithms
                                 // only for printing purposes
};

#endif