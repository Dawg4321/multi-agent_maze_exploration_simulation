#ifndef MULTIROBOT_C_H
#define MULTIROBOT_C_H

#include "MultiRobot.h"

class MultiRobot_C: virtual public MultiRobot{
    public:

        MultiRobot_C(); // constructor for multi-robot collision
        virtual ~MultiRobot_C(); // virtual destructor to ensure child destructor is called during "delete" to base class pointer
    
    protected:
        // protected functions:
        
        // ** General Purpose Functions **
        Coordinates getTarget2Pathfind() { return target_2_pathfind; } // gets the robot's current pathfinding target

        // ** Robot -> Master Communication Functions **
        void requestMove2Cell(Coordinates target_cell); // checks if a cell is occupied by another robot
        void requestGetMap(); // requests a map from current robot position to target cell
        
        // ** Master -> Robot Communication Stub Functions **
        int handleCollisionRequest(Message* response, int current_status); // function to handle Master Request Message
                                                                           // must be used with RobotMaster::handleMasterRequest
        int handleCollisionResponse(Message* response, int current_status); // handles response for collision messages
                                                                            // must be used with RobotMaster::handleMasterResponse
    private:
        Coordinates target_2_pathfind; // coordinate of target cell which robot must generate path to after "job swap"
};

#endif