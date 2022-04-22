#include "RobotMaster_Greedy.h"

RobotMaster_Greedy::RobotMaster_Greedy(){

}

RobotMaster_Greedy::~RobotMaster_Greedy(){

}

// function to handle if there is two robots who are reserving the same cell
// this is the "greedy" implementation (e.g. if two robots are reserving the same cell, let the closer robot keep the reservation)
void RobotMaster_Greedy::handleAlreadyReservedCell(RobotInfo* current_robot, RobotInfo* reserving_robot, m_reserveCellRequest* request_data, m_reserveCellResponse* response_data, Coordinates target_cell){
    
    if(reserving_robot->planned_path.size() <= request_data->planned_path.size()){ // if the target cell has already been reserved and the reserving robot is closer to the target
        response_data->target_cell = target_cell; // adding target cell to response so robot knows which cell is already reserved by another robot
        response_data->cell_reserved = false; // adding information about cell not being reserved to response
    }
    else{ // if the current robot is closer to the target cell than the current reserving robot
        // reserving target cell + updating current planned path
        current_robot->planned_path = request_data->planned_path; // setting planned path to robot's target
        current_robot->robot_target = target_cell; // gathering robot's target cell 

        reserving_robot->robot_target = NULL_COORDINATE; // setting robot target to an invalid value as previous robot now has no currently reserved cell
        
        /*if(reserving_robot->robot_moving){ // if the previously reserving robot is currently moving into a cell
            if(reserving_robot->planned_path.size() > 1){
                reserving_robot->planned_path.erase(reserving_robot->planned_path.begin() + 1, reserving_robot->planned_path.end());
            }
            else{

            }
        }
        else{
            reserving_robot->planned_path.clear();
        }*/

        updateRobotState(2, reserving_robot->Robot_Message_Reciever); // tell previous reserving robot to find a new target

        response_data->cell_reserved = true; // adding information about cell being reserved to response
    }

    return;
}