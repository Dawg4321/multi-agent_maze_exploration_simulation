#ifndef ROBOTMASTER_GREEDY_H
#define ROBOTMASTER_GREEDY_H

#include "RobotMaster_CellReservation.h"

class RobotMaster_Greedy: virtual public RobotMaster_CellReservation{ // child class for greedy reservation strategy
    protected:
        RobotMaster_Greedy();
        ~RobotMaster_Greedy();
        
        void handleAlreadyReservedCell(RobotInfo* current_robot, RobotInfo* reserving_robot, m_reserveCellRequest* request_data, m_reserveCellResponse* response_data, Coordinates target_cell); // implementation of greedy strategy
};

#endif