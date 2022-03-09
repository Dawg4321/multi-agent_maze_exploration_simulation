#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <deque>
#include <string>

#include "GridGraph.h"
#include "Coordinates.h"

class Robot{ // parent class used as a template for further robot implementations for simulation purposes
    public:
        // ** Robot Constructors **
        Robot(unsigned int x, unsigned int y, unsigned int xsize, unsigned int ysize);
 
        // ** Destructor **
        ~Robot();

        // ** Robot Loop Function **
        virtual void robotLoop(GridGraph* maze) = 0; // function used by robots to run exploration behaviour

        // ** Low Level Robot Operations **
        std::vector<bool> scanCell(GridGraph* maze); // scans content of robots current cell using maze information
        bool move2Cell(int direction); // moves robot to cell if possible in specified direction
        bool move2Cell(Coordinates destination); // moves robot to neighbouring cell using neighbouring cell coordinates

        // ** Path-Finding Functions **
        bool pf_BFS(int x, int y); // modifies planned path with the fastest path to a specified location
        bool BFS_pf2NearestUnknownCell(std::deque<Coordinates>* ret_vector); // modifies planned path with fastest path to the closest unknown cell on robot's local map
        
        std::vector<Coordinates> getValidNeighbours(unsigned int x, unsigned  int y); // gathers valid neighbours of a cell
                                                                                      // used in pathfinding functions
        // **Printing Functions**
        bool printRobotMaze();
        void printRobotXMap();
        void printRobotYMap();
        void printRobotNodes();

    protected:
        unsigned int x_position; // x position within cells 
        unsigned int y_position; // y position within cells

        int number_of_unexplored; // number of unexplored cells encountered

        std::deque<Coordinates> planned_path; 
        
        GridGraph* LocalMap; // local_map maintained by robot of areas explored

    private:
        unsigned int maze_xsize; // size of maze
        unsigned int maze_ysize; // this is not used by exploration algorithms
                                 // only for printing purposes

};

#endif