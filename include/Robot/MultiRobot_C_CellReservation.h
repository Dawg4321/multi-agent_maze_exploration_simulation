#ifndef MULTIROBOT_C_IE_H
#define MULTIROBOT_C_IE_H

#include "MultiRobot_C.h"
#include "MultiRobot_CellReservation.h"

class MultiRobot_C_CellReservation: public MultiRobot_CellReservation, public MultiRobot_C{
    public:
        MultiRobot_C_CellReservation(int x, int y, RequestHandler* r, unsigned int xsize, unsigned int ysize); // constructor for multi-robot exploration purposes

        // ** Robot loop function **
        void robotLoop(GridGraph* maze); // loop used by robot for operation
        void robotSetUp(); // function used by robot once before robot begins its loop function
        int robotLoopStep(GridGraph* maze); // function used within each iteration of a robot's loop
                                            // returns the value of the robot's status after iteration

        int robotLoopStepforSimulation(GridGraph* maze); // robot loop step used for simulation to allow for turn delays based off specific requests
                                                         // must be used with turn based simulation system  
        void computeRobotStatus(GridGraph* maze); // function which computes a function based off the robot's current status

    protected:
        // ** Master -> Robot Communication Stub Functions ** 
        int handleMasterResponse(Message* response, int current_status); // function to handle Master Response Messages
        int handleMasterRequest(Message* response, int current_status); // function to handle Master Request Messages
                                                                            // must be overriden to implement stub functions from parent classes
}; 

#endif