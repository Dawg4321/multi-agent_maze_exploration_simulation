#ifndef ROBOTMASTER_IE_H
#define ROBOTMASTER_IE_H

#include "RobotMaster_CellReservation.h"

class RobotMaster_IE: virtual public RobotMaster_CellReservation{
    protected:
        RobotMaster_IE();
        ~RobotMaster_IE();
        
        void handleAlreadyReservedCell(RobotInfo* current_robot, RobotInfo* reserving_robot, m_reserveCellRequest* request_data, m_reserveCellResponse* response_data, Coordinates target_cell);

};

#endif