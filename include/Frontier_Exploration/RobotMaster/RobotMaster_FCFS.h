#ifndef ROBOTMASTER_FCFS_H
#define ROBOTMASTER_FCFS_H

#include "RobotMaster_CellReservation.h"

class RobotMaster_FCFS: virtual public RobotMaster_CellReservation{ // child class for first-come-first-served reservation strategy
    protected:
        RobotMaster_FCFS();
        ~RobotMaster_FCFS();

        void handleAlreadyReservedCell(RobotInfo* current_robot, RobotInfo* reserving_robot, m_reserveCellRequest* request_data, m_reserveCellResponse* response_data, Coordinates target_cell); // implementation of FCFS strategy
};

#endif