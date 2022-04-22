#ifndef ROBOTMASTER_IE_V2_H
#define ROBOTMASTER_IE_V2_H

#include "RobotMaster.h"

class RobotMaster_IE_V2: virtual public RobotMaster{
    protected:
        RobotMaster_IE_V2();
        ~RobotMaster_IE_V2();

        void reserveCellRequest(Message* request); // handles reserve cell requests

         RobotInfo* isCellReserved(Coordinates* target_cell, unsigned int robot_id); // checks if cell is already a target of another robot using parent's "tracked_robots" vector
};

#endif