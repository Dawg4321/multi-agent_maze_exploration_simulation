#ifndef ROBOTMASTER_NC_GREEDY_H
#define ROBOTMASTER_NC_GREEDY_H

#include "RobotMaster_Greedy.h"

class RobotMaster_NC_Greedy: public RobotMaster_Greedy{
    public:
        RobotMaster_NC_Greedy(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_NC_Greedy();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications
};

#endif