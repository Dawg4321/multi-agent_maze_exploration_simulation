#ifndef ROBOTMASTER_C_IE_V2_H
#define ROBOTMASTER_C_IE_V2_H

#include "RobotMaster_IE_V2.h"
#include "RobotMaster_C.h"

class RobotMaster_C_IE_V2: public RobotMaster_IE_V2, public RobotMaster_C{
    public:
        RobotMaster_C_IE_V2(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_C_IE_V2();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications
};

#endif