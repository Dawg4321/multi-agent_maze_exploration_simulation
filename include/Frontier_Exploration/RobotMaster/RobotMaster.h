#ifndef ROBOTMASTER_H
#define ROBOTMASTER_H

#include <vector>
#include <map>
#include <string.h>
#include <iostream>
#include <fmt/format.h>
#include <fstream>

#include "GridGraph.h"
#include "Coordinates.h"
#include "RequestHandler.h"

#include "json.hpp" // using json.hpp from https://github.com/nlohmann/json
                    // this library is used to export tracked data into a json format

using json = nlohmann::json; // simplifying namespace so "json" can be used instead of "nlohmann::json" when declaring json objects


struct RobotInfo{ // structure to track information of various robots in the swarm

    unsigned int robot_id; // tracks the id of a robot in order for robot differentiation
   
    RequestHandler* Robot_Message_Reciever; // pointer to request handler for messages from RobotMaster to Robot

    Coordinates robot_position; // tracks the current position of a robot in the maze
    
    Coordinates starting_position; // tracks the starting location of the robot

    std::deque<Coordinates> planned_path; // path which robot plans to take to frontier cell

    bool robot_moving; // boolean to determine whether a robot is in the process of moving to a cell

    Coordinates robot_target; // target frontier cell which the robot is travelling to
};

class RobotMaster{ // abstract base class for all supervisor implementations
    public:
        RobotMaster(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        virtual ~RobotMaster();
        
        // ** Supervisor Stub Operation Functions **
        void runRobotMaster(); // runs supervisor
        void robotMasterSetUp(); // function to initialize RobotMaster before receiving requests
        bool receiveRequests();  // recieves and decodes request information from imcoming request
                                 // returns false until all cells have been explored
        // ** print functions **
        void printRequestInfo(Message* Request); // prints information on outcome of receieve request
        std::string printGlobalMap(); // prints global map with robot locations

        // ** setters and getters **
        void setGlobalMap(GridGraph* g); // sets global map with new map value 
        GridGraph* getGlobalMap() { return GlobalMap; } // gets global map
        
        // ** General Purpose Functions **
        int getNumRequestsinQueue(){ return Message_Handler->getNumberofMessages(); } // returns number of requests in RobotMaster's Queue
        int getNumberofRobots() { return num_of_robots; } // gets number of robots added to the system

        // ** Metric Tracking Functions **
        json getRequestInfo(){ return RequestInfo; } // gets a copy of the json containing information on transactions handled by the RobotMaster class
        void clearRequestInfo(){ RequestInfo.clear(); } // clears the contents of the RequestInfo json


    protected:
        // protected functions:

        // ** Supervisor Stub Operation Functions **
        virtual void handleIncomingRequest(Message* m) = 0; // processes all requests except shutdown notifications
                                                            // varies depending on implemented functions
        // ** Request Handling Functions **
        // these are effectively wrapper stub functions for other functions to unpack data for implemented funtions
        void shutDownRequest(Message* request); // calls a function to remove a robot from the syste
        void addRobotRequest(Message* request); // calls a function to add a robot to the system
        void updateGlobalMapRequest(Message* request); // calls a function to udate global map
        void updateRobotLocationRequest(Message* request); // calls a function update a robot's position

        // ** Request Functions **
        // these are the actual implemented functions used by stub wrapper functions
        virtual unsigned int addRobot(unsigned int x, unsigned int y, RequestHandler* r); // adds robots to tracked_robots t
                                                                                          // this is important to allow for the robot to be synchronized by the control system
        void removeRobot(unsigned int robot_id); // removes robot from tracked_robots
        virtual void updateGlobalMap(unsigned int* id, std::vector<bool>* connections, Coordinates* C); // updates global map with information from robot scan
        virtual void updateRobotLocation(unsigned int* id, Coordinates* C); // updates the location of a robot to the location specified
        
        // ** Order message functions **
        // these are functions used to facilitate order requests on robots
        void updateAllRobotState(int status); // updates the state of all robots to the specified value
        void updateRobotState(int status, RequestHandler* Target_Robot_Receiver); // send message to robot to update state

        // ** General Purpose Functions **  
        RequestHandler* getTargetRequestHandler(unsigned int target_id); // gets a request handler for a specific robot
        RobotInfo* getRobotInfo(unsigned int id); // gets pointer to robot info of a robot based on its id

        // ** Printing Functions **
        bool checkIfOccupied(unsigned int x, unsigned int y, unsigned int* ret_variable); // checks if a cell is occupied by a robot
                                                                                          // this is used for printing
        // ** GlobalMap Functions **
        // these functions utilise the global map in one form or another
        void gatherPortionofMap(Coordinates curr_node, Coordinates neighbour_node, std::vector<Coordinates>* map_nodes, std::vector<std::vector<bool>>* map_connections, std::vector<char>* node_status); // generates portion of map to be transfered to robot using two final vectors as return values 
        std::vector<Coordinates> getSeenNeighbours(unsigned int x, unsigned  int y); // Gets explored and seen neighbours to a node      
        std::vector<bool> getNodeEdgeInfo(Coordinates* C); // gets connection information surrounding a node
                                                           // [0] = north edge
                                                           // [1] = south edge
                                                           // [2] = east edge
                                                           // [3] = west edge
        // ** Metric Tracking Functions **
        void exportRequestInfo2JSON(m_genericRequest* request, m_genericRequest* response, unsigned int request_id); // exports information regarding a recieved request to a json file
        void clearTargetCell(unsigned int* robot_id); // removes target cell from robot

        // protected data members:

        GridGraph* GlobalMap; // Supervisor's Global Map of maze

        std::vector<RobotInfo> tracked_robots; // vector to track information on various robots within maze
        
        int number_of_frontier_cells; // number of unexplored cells encountered by Robots

        const int num_of_robots; // variable which specifies number of robots needed for exploration
                                 // exploration won't begin until enough robots have been added

        unsigned int num_of_receieve_transactions; // tracks the number of incoming transactions handled
        
        bool accepting_requests; // boolean to track whether robotmaster is receiving requests
                                 // true = accept requests, false = ignore all requests except shut down request
        private:
            RequestHandler* Message_Handler; // pointer to request handler shared by all Robots and a RobotMaster objects
                                             // private as all messaging handling done by stub functions

            json RequestInfo; // json containing information regarding each request

            unsigned int num_of_added_robots = 0; // counter to track number of robots added to system

            unsigned int maze_xsize; // size of maze
            unsigned int maze_ysize; // this is only used for printing purposes
};

#endif