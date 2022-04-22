#ifndef ROBOTMASTER_IE_V2_H
#define ROBOTMASTER_IE_V2_H

#include "RobotMaster_CellReservation.h"

class RobotMaster_IE_V2: virtual public RobotMaster_CellReservation{
    protected:
        RobotMaster_IE_V2();
        ~RobotMaster_IE_V2();

        void handleAlreadyReservedCell(RobotInfo* current_robot, RobotInfo* reserving_robot, m_reserveCellRequest* request_data, m_reserveCellResponse* response_data, Coordinates target_cell);
};

#endif