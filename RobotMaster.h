#ifndef ROBOTMASTER_H
#define ROBOTMASTER_H

#include <vector>

#include "GridGraph.h"
#include "Coordinates.h"

struct RobotInfo{ // structure to track information of various robots in the swarm
    unsigned int robot_id; // tracks the id of a robot in order for robot differentiation

    char robot_status; // tracks the status of a robot
                       // 0 = stand by
                       // 1 = exploring

    Coordinates robot_position; // tracks the current position of a robot in the graph maze in cartesian form
};

class RobotMaster{
    public:
        RobotMaster(/* args */);
        ~RobotMaster();

        void receiveRequests();  // recieves and decodes request information from imcoming request

        int addRobot(unsigned int x, unsigned int y); // adds robots to tracked_robots This is important to allow for the robot to be synchronized by the control system
        void updateGlobalMap(); // updates global map with information from robot scan
        bool robotMoveRequest(); // function to aid in preventing robot collisions

        int changeRobotState(); // changes robot state using return variable

        void printGlobalMap(); // prints global map with robot locations
        

    private:
        GridGraph GlobalMap; // Global Map 

        std::vector<RobotInfo> tracked_robots; // vector to track information on various robots within maze

        unsigned int id_tracker; // tracks next id to give to robot
};

#endif