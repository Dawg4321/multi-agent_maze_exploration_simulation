#include "SoloRobot.h"

// solo exploration class constructor, calls parent class constructor in initialization list
SoloRobot::SoloRobot(unsigned int x, unsigned int y, unsigned int xsize, unsigned int ysize): Robot(x, y, xsize, ysize) { 
    
}

// solo maze exploration algorithm
void SoloRobot::robotLoop(GridGraph* maze){ 
    
    while(1){ // 1. scan cell
              // 2. find nearest unexplored cell
              // 3. move to nearest unexplored cell
              // repeat

        scanCell(maze); // scan cell 
        
        if(number_of_unexplored == 0){ // no more cells to explore therefore break from scan loop
            printf("Done exploring!\n");
            break;
        }

        printRobotMaze(); // print maze contents for analysis

        BFS_pf2NearestUnknownCell(&planned_path); // move to nearest unseen cell

        for (int i = 0; i < planned_path.size(); i++){ // while there are movements left to be done by robot
            move2Cell(&(planned_path.front())); // gathering next movement from top of the stack
            planned_path.pop_front(); // deleting from stack as movement is completed
        }
    }

    return;
}