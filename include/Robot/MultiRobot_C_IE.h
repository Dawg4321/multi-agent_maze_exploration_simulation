#ifndef MULTIROBOT_C_IE_H
#define MULTIROBOT_C_IE_H

#include "MultiRobot_C.h"
#include "MultiRobot_IE.h"
class MultiRobot_C_IE: public MultiRobot_IE, public MultiRobot_C{
    public:
        MultiRobot_C_IE(unsigned int x, unsigned int y, RequestHandler* r, unsigned int xsize, unsigned int ysize); // constructor for multi-robot exploration purposes

        // ** Robot loop function **
        void robotLoop(GridGraph* maze); // loop used by robot for operation
        void robotSetUp(); // function used by robot once before robot begins its loop function
        int robotLoopStep(GridGraph* maze); // function used within each iteration of a robot's loop
                                            // returns the value of the robot's status after iteration

        int handleMasterResponse(Message* response, int current_status); // function to handle Master Response Messages
        int handleMasterRequest(Message* response, int current_status); // function to handle Master Request Messages

        // ** Robot -> Master Communication Functions **
        // bool requestReserveCell(); // attempts to reserve a cell to explore from the RobotMaster
                                   // if cell to reserve fails, LocalMap is updated with GlobalMap information

        // bool requestMove2Cell(Coordinates target_cell); // checks if a cell is occupied by another robot

    private:
        int robot_status; // tracks status of robot within the robot loop
}; 

#endif