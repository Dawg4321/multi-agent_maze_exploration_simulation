#ifndef ROBOTMASTER_NC_IE_H
#define ROBOTMASTER_NC_IE_H

#include "RobotMaster_Greedy.h"

class RobotMaster_NC_IE: public RobotMaster_Greedy{
    public:
        RobotMaster_NC_IE(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_NC_IE();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications
};

#endif