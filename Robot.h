#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <string>

#include <pthread.h>
#include <semaphore.h>

#include "GridGraph.h"
#include "Coordinates.h"
#include "RequestHandler.h"

class Robot{
    public:
        // **Robot Constructors
        Robot(int x, int y); // contrucutor for solo exploration purposes
        Robot(int x, int y, RequestHandler* r); // constructor for multi-robot exploration purposes

        // ** Robot Loop**
        void startRobot(GridGraph* maze); // loop used by robot to move through maze

        // **Low Level Robot Operations**
        std::vector<bool> scanCell(GridGraph* maze); // scans content of robots current cell using maze information
        bool move2Cell(int direction); // moves robot to cell if possible in specified direction
        bool move2Cell(Coordinates* destination); // moves robot to neighbouring cell using neighbouring cell coordinates

        // **Path-Finding Functions
        bool pf_BFS(int x, int y); // modifies planned path with the fastest path to a specified location
        bool BFS_pf2NearestUnknownCell(std::vector<Coordinates>* ret_vector); // modifies planned path with fastest path to the closest unknown cell on robot's local map
        
        std::vector<Coordinates> getValidNeighbours(unsigned int x, unsigned  int y); // gathers valid neighbours of a cell
                                                                                      // used in pathfinding functions
        // **Robot Algorithms**
        void soloExplore(GridGraph* maze); // algorithm used when robot is exploring alone
        void multiExplore(GridGraph* maze); // algorithm used when robot is exploring using RobotMaster

        // **Printing Functions**
        bool printRobotMaze();
        void printRobotXMap();
        void printRobotYMap();
        void printRobotNodes();

        unsigned int getID() { return id;}

    private:
        unsigned int x_position; // x position within cells 
        unsigned int y_position; // y position within cells

        int number_of_unexplored; // number of unexplored cells encountered

        unsigned int id; // robot id assigned by controller

        std::vector<Coordinates> planned_path; 
        
        GridGraph LocalMap; // local_map maintained by robot of areas explored

        RequestHandler* Message_Handler; // pointer to request handler shared by all Robot objects

        unsigned int maze_xsize; // size of maze
        unsigned int maze_ysize; // this is not used by exploration algorithms
                                 // only for printing purposes
};

#endif