#ifndef MULTIROBOT_H
#define MULTIROBOT_H

#include <pthread.h>
#include <semaphore.h>

#include "Robot.h"
#include "RequestHandler.h"
#include "RequestTemplates.h"

#define s_exit_loop -2
#define s_shut_down -1
#define s_stand_by 0
#define s_scan_cell 1
#define s_pathfind 2
#define s_move_robot 3

class MultiRobot: public Robot{
    public:

        MultiRobot(unsigned int x, unsigned int y, RequestHandler* r, unsigned int xsize, unsigned int ysize); // constructor for multi-robot exploration purposes
        virtual ~MultiRobot(); // virtual destructor to ensure child destructor is called during "delete" to base class pointer
        
        bool move2Cell(Coordinates destination); // move robot operation used when exploring using controller

        // ** Robot Loop Function **  
        virtual void robotLoop(GridGraph* maze) = 0; // loop used by robot to move through maze

        // ** General Purpose Functions **
        unsigned int getID() { return id;} // returns robot id
        void updateLocalMap(std::vector<Coordinates>* map_info, std::vector<std::vector<bool>>* edge_info, std::vector<char>* map_status); // updates robot's map with information from vectors

        // ** Robot -> Master Communication Functions **
        void assignIdFromMaster(); // gets an ID from a RobotMaster using a message

        void requestShutDown(); // sends shutdown notification to RobotMaster
                                // TODO: change to generic update status request                 
        void requestReserveCell(); // attempts to reserve a cell to explore from the RobotMaster
                                    // if cell to reserve fails, LocalMap is updated with GlobalMap information
        bool requestMove2Cell(Coordinates target_cell); // checks if a cell is occupied by another robot

        void requestGlobalMapUpdate(std::vector<bool> connection_data); // updating Global Map information of master with connection data

        void requestRobotLocationUpdate(); // updates robot position to Robot Master after movement has been complete

        // ** Master -> Robot Communication Functions **
        int getMessagesFromMaster(int status); // handles any messages master has sent 
        
        int  handleMasterRequest(Message* request, int current_status); // function to handle Master Request Message
        int  handleMasterResponse(Message* response, int current_status); // function to handle Master Response Messages

        bool isResponseStale(int transaction_id); // simple function to check if there is an outstanding response of a given transaction id
        void makeResponseStale(int transaction_id); // removes entry of transaction from valid_response (e.g. making it stale)
       
    protected:
        unsigned int id; // robot id assigned to robot by robot controller

        int robot_status; // controls robot execution within robot loop

        int transaction_counter; // counts the number of sent transactions executed
                                 // also used to assign transation id to sent messages to allow for response identification

        std::vector<int> valid_responses; // vector to track id of transactions which require a response
                                          // if a response is recieved that is not tracked in here, it must be stale

        RequestHandler* Robot_2_Master_Message_Handler; // pointer to request handler shared by all Robot objects
        RequestHandler* Master_2_Robot_Message_Handler;

        sem_t* response_sem; // semaphores to be used for inter-thread synchornization when passing messages
        sem_t* acknowledgement_sem;
}; 

#endif