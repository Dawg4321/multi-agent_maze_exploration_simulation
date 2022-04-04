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

         void move2CellRequest(Message* request);

    private:
        std::vector<std::vector<CollisionInfo>> CollisionMatrix; // vector used to track which robot is reserving each cell
};

#endif