#include "RobotMaster.h"

RobotMaster::RobotMaster(){
    id_tracker = 0;
}

RobotMaster::~RobotMaster(){
    
}

int RobotMaster::addRobot(unsigned int x, unsigned int y){
    
    id_tracker++; // incrementing inorder to determine next id to give a robot

    RobotInfo temp; // buffer to store robot info before pushing it to the tracked_robots vecto

    temp.robot_id = id_tracker; // assigning id to new robot entry 
    temp.robot_position.x = x;  // assigning position to new robot entry 
    temp.robot_position.y = y;
    temp.robot_status = 0;      // updating current robot status to 0 to leave it on stand by
    
    tracked_robots.push_back(temp); // adding robot info to tracked_robots

    return id_tracker; // returning id to be assigned to the robot which triggered this function
}

void RobotMaster::updateGlobalMap(){

}