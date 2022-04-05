#ifndef MULTIROBOT_IE_H
#define MULTIROBOT_IE_H

#include "MultiRobot.h"

class MultiRobot_IE: virtual public MultiRobot{
    public:

        MultiRobot_IE(); // constructor for multi-robot collision
        virtual ~MultiRobot_IE(); // virtual destructor to ensure child destructor is called during "delete" to base class pointer

        void requestReserveCell(); // attempts to reserve a cell to explore from the RobotMaster
                                   // if cell to reserve fails, LocalMap is updated with GlobalMap information

        int handleCellReserveResponse(Message* response, int current_status); // handles response for collision messages
                                                                              // must be used with RobotMaster::handleMasterRequest

        void updateLocalMap(std::vector<Coordinates>* map_info, std::vector<std::vector<bool>>* edge_info, std::vector<char>* map_status); // updates robot's map with information from vectors
};

#endif