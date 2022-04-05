#ifndef MULTIROBOT_C_H
#define MULTIROBOT_C_H

#include "MultiRobot.h"

class MultiRobot_C: virtual public MultiRobot{
    public:

        MultiRobot_C(); // constructor for multi-robot collision
        virtual ~MultiRobot_C(); // virtual destructor to ensure child destructor is called during "delete" to base class pointer

        void requestMove2Cell(Coordinates target_cell); // checks if a cell is occupied by another robot

        int handleCollisionResponse(Message* response, int current_status); // handles response for collision messages
                                                                            // must be used with RobotMaster::handleMasterRequest
};

#endif