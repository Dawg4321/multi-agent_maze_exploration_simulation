#ifndef ROBOTMASTER_IE_H
#define ROBOTMASTER_IE_H

#include "RobotMaster.h"

struct ReservationInfo{ // structure to track information of each cell within the maze
                        // used in conjuction with GridGraph struct

    unsigned int reserved;  // tracks whether the cell has been reserved for exploration purposes
};

class RobotMaster_IE: virtual public RobotMaster{
    protected:
        RobotMaster_IE(unsigned int xsize, unsigned int ysize);
        ~RobotMaster_IE();

        void reserveCellRequest(Message* request); // handles reserve cell requests
        
        void unreserveCell(unsigned int robot_id); // unreserves the current target cell of a robot
                                                           
        void updateGlobalMap(unsigned int* id, std::vector<bool>* connections, Coordinates* C); // overloading update global map as cell must be unreserved after scan has been completed

        bool isCellReserved(Coordinates* target_cell); // checks if cell is already a target of another robot using parent's "tracked_robots" vector
    
    /* private:
        std::vector<std::vector<ReservationInfo>> ReservationMatrix; // vector used to track which robot is reserving each cell*/
};

#endif