#ifndef ROBOTMASTER_C_IE_H
#define ROBOTMASTER_C_IE_H

#include "RobotMaster_IE.h"
#include "RobotMaster_C.h"

class RobotMaster_C_IE: public RobotMaster_IE, public RobotMaster_C{
    public:
        RobotMaster_C_IE(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_C_IE();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications
};

#endif