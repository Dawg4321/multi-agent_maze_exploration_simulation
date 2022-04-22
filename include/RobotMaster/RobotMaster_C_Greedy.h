#ifndef ROBOTMASTER_C_GREEDY_H
#define ROBOTMASTER_C_GREEDY_H

#include "RobotMaster_Greedy.h"
#include "RobotMaster_C.h"

class RobotMaster_C_Greedy: public RobotMaster_Greedy, public RobotMaster_C{
    public:
        RobotMaster_C_Greedy(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_C_Greedy();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications
};

#endif