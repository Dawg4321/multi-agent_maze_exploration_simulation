#ifndef SOLOROBOT_H
#define SOLOROBOT_H

#include <Robot.h>

class SoloRobot: public Robot{
    public:
        SoloRobot(unsigned int x, unsigned int y, unsigned int xsize, unsigned int ysize); // contrucutor for solo exploration purposes
        
        void robotLoop(GridGraph* maze); // algorithm used for solo maze exploration
}; 

#endif