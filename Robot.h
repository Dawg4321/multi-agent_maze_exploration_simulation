#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <deque>
#include <string>

#include <pthread.h>
#include <semaphore.h>

#include "GridGraph.h"
#include "Coordinates.h"
#include "RequestHandler.h"

class Robot{
    public:
        // **Robot Constructors **
        Robot(int x, int y, int xsize, int ysize); // contrucutor for solo exploration purposes
        Robot(int x, int y, RequestHandler* r, unsigned int xsize, unsigned int ysize); // constructor for multi-robot exploration purposes
        
        // ** Destructor **
        ~Robot();
        // ** Robot Loop**
        void multiRobotLoop(GridGraph* maze); // loop used by robot to move through maze

        // ** Low Level Robot Operations **
        std::vector<bool> scanCell(GridGraph* maze); // scans content of robots current cell using maze information
        bool move2Cell(int direction); // moves robot to cell if possible in specified direction
        bool move2Cell(Coordinates* destination); // moves robot to neighbouring cell using neighbouring cell coordinates
        bool m_move2Cell(Coordinates* destination); // move robot operation used when exploring using controller

        // ** Path-Finding Functions **
        bool pf_BFS(int x, int y); // modifies planned path with the fastest path to a specified location
        bool BFS_pf2NearestUnknownCell(std::deque<Coordinates>* ret_vector); // modifies planned path with fastest path to the closest unknown cell on robot's local map
        
        std::vector<Coordinates> getValidNeighbours(unsigned int x, unsigned  int y); // gathers valid neighbours of a cell
                                                                                      // used in pathfinding functions
        // ** Robot Algorithms **
        void soloExplore(GridGraph* maze); // algorithm used when robot is exploring alone
        void multiExplore(GridGraph* maze); // algorithm used when robot is exploring using RobotMaster

        // ** General Purpose Functions **
        void assignIdFromMaster(); // gets an ID from a RobotMaster using a message
        int getRequestsFromMaster(int status); // checks if master wants to change current status of robot
        
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

        std::deque<Coordinates> planned_path; 
        
        GridGraph* LocalMap; // local_map maintained by robot of areas explored

        RequestHandler* Robot_2_Master_Message_Handler; // pointer to request handler shared by all Robot objects
        RequestHandler* Master_2_Robot_Message_Handler;

        sem_t* response_sem; // semaphores to be used for inter-thread synchornization when passing messages
        sem_t* acknowledgement_sem;

        unsigned int maze_xsize; // size of maze
        unsigned int maze_ysize; // this is not used by exploration algorithms
                                 // only for printing purposes

};

#endif