#ifndef ROBOTMASTER_NC_UI_H
#define ROBOTMASTER_NC_UI_H

#include "RobotMaster.h"

class RobotMaster_NC: public RobotMaster{
    public:
        RobotMaster_NC(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_NC();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications

        void updateRobotLocation(unsigned int* id, Coordinates* C); // updates the location of a robot to the location specified
};

#endif