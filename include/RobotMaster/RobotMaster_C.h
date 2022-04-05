#ifndef ROBOTMASTER_C_H
#define ROBOTMASTER_C_H

#include "RobotMaster.h"

struct CollisionInfo{
    unsigned int occupying_robot; // stores the id of a robot which is currently occupying a cell
};

class RobotMaster_C: virtual public RobotMaster{
    protected:
        RobotMaster_C(unsigned int xsize, unsigned int ysize);
        ~RobotMaster_C();

        void move2CellRequest(Message* request); // attempts to reserve a cell for movement
                                                 // this prevents two robots from moving to the same cell at the same time

        void updateRobotLocation(unsigned int* id, Coordinates* C); // updates the location of a robot to the location specified

        unsigned int addRobot(unsigned int x, unsigned int y, RequestHandler* r);

    private:
        std::vector<std::vector<CollisionInfo>> CollisionMatrix; // vector used to track which robot is reserving each cell for movement purposes
};

#endif