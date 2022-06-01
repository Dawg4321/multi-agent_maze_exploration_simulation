#ifndef ROBOTMASTER_NC_FCFS_H
#define ROBOTMASTER_NC_FCFS_H

#include "RobotMaster_FCFS.h"

class RobotMaster_NC_FCFS: public RobotMaster_FCFS{
    public:
        RobotMaster_NC_FCFS(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_NC_FCFS();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications
};

#endif