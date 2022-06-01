#ifndef RobotMaster_C_FCFS_H
#define RobotMaster_C_FCFS_H

#include "RobotMaster_FCFS.h"
#include "RobotMaster_C.h"

class RobotMaster_C_FCFS: public RobotMaster_FCFS, public RobotMaster_C{
    public:
        RobotMaster_C_FCFS(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize);
        ~RobotMaster_C_FCFS();

        void handleIncomingRequest(Message* m); // processes all requests except shutdown notifications
};

#endif