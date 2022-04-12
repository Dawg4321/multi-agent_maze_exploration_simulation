#ifndef ROBOTMASTER_C_H
#define ROBOTMASTER_C_H

#include "RobotMaster.h"

class RobotMaster_C: virtual public RobotMaster{
    protected:
        RobotMaster_C(unsigned int xsize, unsigned int ysize);
        ~RobotMaster_C();

        void move2CellRequest(Message* request); // attempts to reserve a cell for movement
                                                 // this prevents two robots from moving to the same cell at the same time
        
        void getMapRequest(Message* request); // returns a map to a specified cell from robot's position
                                              // gives the robot the opportunity to create a path to a new target cell

        void gatherMap2Target(Coordinates current_node, Coordinates target_node, std::vector<Coordinates>* map_nodes, std::vector<std::vector<bool>>* map_connections, std::vector<char>* node_status);

        void updateRobotMovement(unsigned int* id, Coordinates* C); // updates the next_move value for given robot (e.g. "reserve" the cell so no other robot will move into it until the movement has been completed)

        RobotInfo* checkForCollision(Coordinates* movement_cell, unsigned int robot_id); // checks if there a collision between passed robot and any other robot. Returns pointer robotinfo of collision causing robot

        unsigned int addRobot(unsigned int x, unsigned int y, RequestHandler* r);

        void setTargetCellRequest(Coordinates targetcell, unsigned int target_robot);
};

#endif