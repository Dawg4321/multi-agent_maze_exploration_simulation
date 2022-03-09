#ifndef MULTIROBOT_H
#define MULTIROBOT_H

#include <pthread.h>
#include <semaphore.h>

#include "Robot.h"
#include "RequestHandler.h"

class MultiRobot: public Robot{
    public:

        MultiRobot(unsigned int x, unsigned int y, RequestHandler* r, unsigned int xsize, unsigned int ysize); // constructor for multi-robot exploration purposes
        ~MultiRobot();
        
        bool move2Cell(Coordinates destination); // move robot operation used when exploring using controller

        void robotLoop(GridGraph* maze); // loop used by robot to move through maze

        // ** Robot Algorithms **    
        void multiExplore(GridGraph* maze); // algorithm used when robot is exploring using RobotMaster

        // ** General Purpose Functions **
        unsigned int getID() { return id;} // returns robot id
        void updateLocalMap(std::vector<Coordinates>* map_info, std::vector<std::vector<bool>>* edge_info, std::vector<char>* map_status); // updates robot's map with information from vectors

        // ** Robot -> Master Communication Functions **
        void assignIdFromMaster(); // gets an ID from a RobotMaster using a message

        void requestShutDown(); // sends shutdown notification to RobotMaster
                                // TODO: change to generic update status request                 
        bool requestReserveCell(); // attempts to reserve a cell to explore from the RobotMaster
                                    // if cell to reserve fails, LocalMap is updated with GlobalMap information
        bool requestMove2Cell(Coordinates target_cell); // checks if a cell is occupied by another robot

        void requestGlobalMapUpdate(std::vector<bool> connection_data); // updating Global Map information of master with connection data

        void requestRobotLocationUpdate(); // updates robot position to Robot Master after movement has been complete

        // ** Master -> Robot Communication Functions **
        int getRequestsFromMaster(int status); // checks if master wants to change current status of robot

    protected:
        unsigned int id; // robot id assigned by controller

        RequestHandler* Robot_2_Master_Message_Handler; // pointer to request handler shared by all Robot objects
        RequestHandler* Master_2_Robot_Message_Handler;

        sem_t* response_sem; // semaphores to be used for inter-thread synchornization when passing messages
        sem_t* acknowledgement_sem;
}; 

#endif