#ifndef ROBOTMASTER_C_H
#define ROBOTMASTER_C_H

#include <stdexcept>

#include "RobotMaster.h"

class RobotMaster_C: virtual public RobotMaster{
    protected:
        RobotMaster_C();
        ~RobotMaster_C();
        
        // ** Request Handling Functions **
        // these are a combination of a wrapper stub function with functionalities implemented
        void move2CellRequest(Message* request); // attempts to reserve a cell for movement using incoming message
                                                 // this prevents two robots from moving to the same cell at the same time
        void getMapRequest(Message* request); // Calls function to return a map to a specified cell from robot's position
                                              // gives the robot the opportunity to create a path to a new target cell

        // ** Request Functions **
        void gatherMap2Target(Coordinates current_node, Coordinates target_node, std::vector<Coordinates>* map_nodes, std::vector<std::vector<bool>>* map_connections, std::vector<char>* node_status); // gahters a portion of the map from robot position to target

        // ** General Purpose Functions **  
        RobotInfo* checkForCollision(Coordinates* movement_cell, unsigned int robot_id); // checks if there a collision between passed robot and any other robot. Returns pointer robotinfo of collision causing robot
        bool isRobotMoving(Coordinates C, unsigned int robot_id); // checks to see if another robot is in the process of to move to the cell
        void setTargetCellRequest(Coordinates targetcell, unsigned int target_robot);
};

#endif