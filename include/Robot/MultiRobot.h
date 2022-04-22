#ifndef MULTIROBOT_H
#define MULTIROBOT_H

#include "Robot.h"
#include "RequestHandler.h"

#define s_exit_loop -2
#define s_shut_down -1
#define s_stand_by 0
#define s_scan_cell 1
#define s_pathfind 2
#define s_move_robot 3
#define s_compute_move 4
#define s_pathfind2target 5

class MultiRobot: public Robot{
    public:

        MultiRobot(int x, int y, RequestHandler* r, unsigned int xsize, unsigned int ysize); // constructor for multi-robot exploration purposes
        virtual ~MultiRobot(); // virtual destructor to ensure child destructor is called during "delete" to base class pointer
        
        bool move2Cell(Coordinates destination); // move robot operation used when exploring using controller

        // ** Robot Loop Functions **  
        virtual void robotLoop(GridGraph* maze) = 0; // loop used by robot to move through maze
        virtual void robotSetUp() = 0; // function used by robot once before robot begins its loop function
        virtual int robotLoopStep(GridGraph* maze) = 0; // function used within each iteration of a robot's loop
                                                         // returns the value of the robot's status after iteration
        virtual int robotLoopStepforSimulation(GridGraph* maze) {} // robot loop step used for simulation to allow for turn delays based off specific requests
                                                                     // must be used with turn based simulation system

        void updateLocalMap(std::vector<Coordinates>* map_info, std::vector<std::vector<bool>>* edge_info, std::vector<char>* map_status); // updates robot's map with information from vectors

        // ** General Purpose Functions **
        unsigned int getID() { return id;} // returns robot id

        // ** Robot -> Master Communication Functions **
        void assignIdFromMaster(); // gets an ID from a RobotMaster using a message

        void requestShutDown(); // sends shutdown notification to RobotMaster
                                // TODO: change to generic update status request                 

        void requestGlobalMapUpdate(std::vector<bool> connection_data); // updating Global Map information of master with connection data

        void requestRobotLocationUpdate(); // updates robot position to Robot Master after movement has been complete

        // ** Master -> Robot Communication Functions **
        int getMessagesFromMaster(int status); // handles any messages master has sent 
        
        virtual int handleMasterRequest(Message* request, int current_status); // function to handle Master Request Message
        virtual int handleMasterResponse(Message* response, int current_status); // function to handle Master Response Messages

        bool isResponseStale(int transaction_id); // simple function to check if there is an outstanding response of a given transaction id
        void makeResponseStale(int transaction_id); // removes entry of transaction from valid_response (e.g. making it stale)

        void computeMove(); // function used to compute move of the robot
        void computeScanCell(GridGraph* maze); // computing scan cell
       
    protected:
        unsigned int id; // robot id assigned to robot by robot controller

        int transaction_counter; // counts the number of sent transactions executed
                                 // also used to assign transation id to sent messages to allow for response identification

        std::vector<int> valid_responses; // vector to track id of transactions which require a response
                                          // if a response is recieved that is not tracked in here, it must be stale

        int last_request_priority; // variable to track allow for priority between different RobotMaster's requests 
                                   // e.g. ensures shut down request should have priority over all other requests (another update status request cannot override a shutdown request)
                                   // this should only be used in getMessagesFromMaster and any other nested functions

        bool accepting_requests; // tracks whether the robot is willing to accept request from RobotMaster
                                 // will be set to false if RobotMaster has told robot to shutdown

        RequestHandler* Robot_2_Master_Message_Handler; // pointer to request handler shared by all Robot objects
        RequestHandler* Master_2_Robot_Message_Handler;

        int robot_status; // tracks status of robot within the robot loop
}; 

#endif