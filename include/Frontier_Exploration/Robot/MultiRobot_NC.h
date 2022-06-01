#ifndef MULTIROBOT_NC_H
#define MULTIROBOT_NC_H

#include "MultiRobot.h"
/* 
    DEPRECATED FUNCTIONALITY USE AT OWN RISK
*/
class MultiRobot_NC: public MultiRobot{
    public:

        MultiRobot_NC(unsigned int x, unsigned int y, RequestHandler* r, unsigned int xsize, unsigned int ysize); // constructor for multi-robot exploration purposes

        // ** Robot loop functions **
        void robotLoop(GridGraph* maze); // loop used by robot for operation
        void robotSetUp(); // function used by robot once before robot begins its loop function
        int robotLoopStep(GridGraph* maze); // function used within each iteration of a robot's loop
                                            // returns the value of the robot's status after iteration
        
        int robotLoopStepforSimulation(GridGraph* maze); // robot loop step used for simulation to allow for turn delays based off specific requests
                                                         // must be used with turn based simulation system  
        void computeRobotStatus(GridGraph* maze); // function which computes a function based off the robot's current status  
};

#endif