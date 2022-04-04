#ifndef ROBOTMASTER_NC_UI_H
#define ROBOTMASTER_NC_UI_H

#include "RobotMaster.h"

class RobotMaster_NC_UI: public RobotMaster{
    public:
        RobotMaster_NC_UI(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_NC_UI();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications
};

#endif