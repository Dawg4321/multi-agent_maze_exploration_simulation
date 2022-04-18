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
        Robot(int x, int y, unsigned int xsize, unsigned int ysize);
 
        // ** Destructor **
        virtual ~Robot(); // virtual destructor to ensure child destructor is called during "delete" to base class pointer

        // ** Robot Loop Function **
        virtual void robotLoop(GridGraph* maze) = 0; // function used by robots to run exploration behaviour
        virtual void robotSetUp() = 0; // function used by robot once before robot begins its loop function
        virtual int robotLoopStep(GridGraph* maze) = 0; // function used within each iteration of a robot's loop
                                                         // returns the value of the robot's status after iteration

        // ** Low Level Robot Operations **
        std::vector<bool> scanCell(GridGraph* maze); // scans content of robots current cell using maze information
        bool move2Cell(int direction); // moves robot to cell if possible in specified direction
        bool move2Cell(Coordinates destination); // moves robot to neighbouring cell using neighbouring cell coordinates

        // ** Path-Finding Functions **
        bool pf_BFS(int x, int y); // modifies planned path with the fastest path to a specified location
        bool BFS_pf2NearestUnknownCell(std::deque<Coordinates>* ret_vector); // modifies planned path with fastest path to the closest unknown cell on robot's local map
        virtual void BFS_noPathFound(); // function which handles if a path is not found
        virtual bool BFS_exitCondition(Coordinates* node_to_test); // function used to determine exit condition from BFS_pf2NearestUnknownCell

        std::vector<Coordinates> getValidNeighbours(int x, int y); // gathers valid neighbours of a cell
                                                                                      // used in pathfinding functions
        // **Printing Functions**
        bool printRobotMaze();
        void printRobotXMap();
        void printRobotYMap();
        void printRobotNodes();

        void setLocalMap(GridGraph* new_map); // passes contents of map in robot's LocalMap

    protected:
        int x_position; // x position within cells 
        int y_position; // y position within cells

        int number_of_unexplored; // number of unexplored cells encountered

        std::deque<Coordinates> planned_path; 
        
        GridGraph* LocalMap; // local_map maintained by robot of areas explored

    private:
        unsigned int maze_xsize; // size of maze
        unsigned int maze_ysize; // this is not used by exploration algorithms
                                 // only for printing purposes

};

#endif