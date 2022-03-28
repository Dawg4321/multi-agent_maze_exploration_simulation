#ifndef ROBOTMASTER_H
#define ROBOTMASTER_H

#include <vector>
#include <pthread.h>
#include <semaphore.h>

#include "GridGraph.h"
#include "Coordinates.h"
#include "RequestHandler.h"
#include "RequestTemplates.h"

#include "json.hpp" // using json.hpp from https://github.com/nlohmann/json
                    // this library is used to export tracked data into a json format

using json = nlohmann::json; // simplifying namespace so "json" can be used instead of "nlohmann::json" when declaring json objects

struct RobotInfo{ // structure to track information of various robots in the swarm
    unsigned int robot_id; // tracks the id of a robot in order for robot differentiation

    int robot_status; // tracks the status of a robot
                       // 0 = stand by
                       // 1 = exploring

    RequestHandler* Robot_Message_Reciever; // pointer to request handler for messages from RobotMaster to Robot

    Coordinates robot_position; // tracks the current position of a robot in the graph maze in cartesian form
};

struct CellInfo{ // structure to track information of each cell within the maze
                 // used in conjuction with GridGraph struct

    unsigned int occupying_robot; // tracks the id of a robot which is occupying a cell
                                  // will be 0 if unoccupied

    unsigned int reserved;  // tracks whether the cell has been reserved for exploration purposes
};

class RobotMaster{
    public:
        RobotMaster(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster();

        // ** Master Operation Function **
        void runRobotMaster();
        bool receiveRequests();  // recieves and decodes request information from imcoming request
                                 // returns false until all cells have been explored
        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications

        // ** Request Handling Functions **
        // these are effectively wrapper functions for other functions to facilitate interthread communication
        void shutDownRequest(Message* request);
        void addRobotRequest(Message* request);
        void updateGlobalMapRequest(Message* request);
        virtual void move2CellRequest(Message* request);
        void reserveCellRequest(Message* request);
        void updateRobotLocationRequest(Message* request);

        // ** General Purpose Functions **
        bool checkIfOccupied(unsigned int x, unsigned int y, unsigned int* ret_variable); // checks if a cell is occupied by a robot

        RequestHandler* getTargetRequestHandler(unsigned int target_id); // gets a request handler for a specific robot

        // ** Tracked_Robots Functions **
        unsigned int addRobot(unsigned int x, unsigned int y, RequestHandler* r); // adds robots to tracked_robots t
                                                                                  // this is important to allow for the robot to be synchronized by the control system
        void removeRobot(unsigned int robot_id); // removes robot from tracked_robots
        
        bool robotMoveCheck(); // function to aid in preventing robot collisions

        void updateRobotLocation(unsigned int* id, Coordinates* C); // updates the location of a robot to the location specified

        void updateAllRobotState(int status); // updates the state of all robots to the specified value

        // ** GlobalMap Functions **
        void updateGlobalMap(unsigned int* id, std::vector<bool>* connections, Coordinates* C); // updates global map with information from robot scan
        
        void gatherPortionofMap(Coordinates curr_node, Coordinates neighbour_node, std::vector<Coordinates>* map_nodes, std::vector<std::vector<bool>>* map_connections, std::vector<char>* node_status); // generates portion of map to be transfered to robot using two final vectors as return values
        
        std::vector<Coordinates> getSeenNeighbours(unsigned int x, unsigned  int y); // Gets explored and seen neighbours to a node
        
        std::vector<bool> getNodeEdgeInfo(Coordinates* C); // gets connection information surrounding a node
                                                           // [0] = north edge
                                                           // [1] = south edge
                                                           // [2] = east edge
                                                           // [3] = west edge
        // ** Metric Tracking Functions **
        void exportRequestInfo2JSON(m_genericRequest* request, m_genericRequest* response, unsigned int request_id); // exports information regarding a recieved request to a json file
        json getRequestInfo(){ return RequestInfo; } // gets a copy of the json containing information on transactions handled by the RobotMaster classs
        void clearRequestInfo(){ RequestInfo.clear(); } // clears the contents of the RequestInfo json

        // ** print functions **
        void printRequestInfo(Message* Request); // prints information on outcome of receieve request
        bool printGlobalMap(); // prints global map with robot locations

    protected:
        GridGraph* GlobalMap; // Global Map of maze

        std::vector<std::vector<CellInfo>> GlobalMapInfo; // vector used to track status of various cells

        std::vector<RobotInfo> tracked_robots; // vector to track information on various robots within maze

        RequestHandler* Message_Handler; // pointer to request handler shared by all Robots and a RobotMaster objects
        
        int number_of_unexplored_cells; // number of unexplored cells encountered by Robots

        unsigned int maze_xsize; // size of maze
        unsigned int maze_ysize; // this is only used for printing purposes

        unsigned int num_of_added_robots; // tracks number of added robots
        
        const int max_num_of_robots; // variable which specifies number of robots needed for exploration
                                     // exploration won't begin until enough robots have been added

        unsigned int num_of_receieve_transactions; // tracks the number of incoming transactions handled

        json RequestInfo; // json containing information regarding each request

        bool accepting_requests; // boolean to track whether robotmaster is receiving requests
                                 // true = accept requests, false = ignore all requests except shut down request
};

#endif