#ifndef ROBOTMASTER_CELLRESERVATION_H
#define ROBOTMASTER_CELLRESERVATION_H

#include "RobotMaster.h"

class RobotMaster_CellReservation: virtual public RobotMaster{ // class which outlines basic functions
    protected:
        RobotMaster_CellReservation();
        ~RobotMaster_CellReservation();

        void reserveCellRequest(Message* request); // handles reserve cell request from a robot

        virtual void handleAlreadyReservedCell(RobotInfo* current_robot, RobotInfo* reserving_robot, m_reserveCellRequest* request_data, m_reserveCellResponse* response_data, Coordinates target_cell) = 0; // handles the case where a cell is already reserved by another robot
                                                      // function which is to be implemented in child classes 

        void reserveCell(RobotInfo* robot_info, std::deque<Coordinates>* path_2_target , Coordinates target_cell); // function to reserve a cell in child classes
        RobotInfo* isCellReserved(Coordinates* target_cell, unsigned int robot_id); // checks if cell is already a target of another robot using parent's "tracked_robots" vector
};

#endif