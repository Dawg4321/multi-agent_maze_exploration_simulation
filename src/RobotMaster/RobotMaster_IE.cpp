#include "RobotMaster_IE.h"

RobotMaster_IE::RobotMaster_IE(unsigned int xsize, unsigned int ysize){

}

RobotMaster_IE::~RobotMaster_IE(){

}

void RobotMaster_IE::reserveCellRequest(Message* request){
    // reserveCell request msg_data layout:
    // [0] = type: (unsigned int*), content: id of robot sending request
    // [1] = type: (Coordinates*), content: target unexplored cell to reseve
    // [2] = type: (Coordinates*), content: neighbouring cell used to enter target cell 
                                            // used to determine what aspect of tree must be sent back in event node has already been explored 
    // return data
    // [0] = type (bool*), content: whether cell has been reserved
    // [1] = type (vector<Coordinates>*), content: coordinates correspoonding to full map down a node path if path has been explored already
    // [2] = type (vector<vector<bool>>*), content: wall information corresponding to cell locations in [1] 
    // [3] = type (vector<char>*), content: information on node status

    // gathering incoming request data
    m_reserveCellRequest* request_data = (m_reserveCellRequest*)request->msg_data;

    unsigned int robot_id = request_data->robot_id;
    Coordinates target_cell = request_data->planned_path.back();                    
    Coordinates neighbouring_cell = request_data->neighbouring_cell;

    // return data allocation
    m_reserveCellResponse* response_data = new m_reserveCellResponse; // response message

    bool cell_reserved; // variable to whether cell reservation is possible

    RobotInfo* reserving_robot = isCellReserved(&target_cell, robot_id); // checking if another robot is reserving the target cell

    // processing if cell can be reserved
    // in this case, vectors allocated in response_data will be modified
    if(GlobalMap->nodes[target_cell.y][target_cell.x] == 1){ // if the target cell has already been explored
        // gathering portion of map outwards from unexplored node to return to robot
        gatherPortionofMap(target_cell, neighbouring_cell, response_data->map_coordinates, response_data->map_connections, response_data->map_status);
        cell_reserved = false; // cell has not been successfully reserved for requesting robot
    }
    else if(reserving_robot == NULL){ // no other robot has reserved the target cell and it has not been explored
        // reserving target cell + updating current planned path
        RobotInfo* robot_info = getRobotInfo(robot_id);// gathering planned_path for exploration
        robot_info->planned_path = request_data->planned_path; // setting planned path to robot's target
        robot_info->robot_target = target_cell; // gathering robot's target cell

        // response data loading
        cell_reserved = true; // cell has been reserved
    }
    else if(reserving_robot->planned_path.size() <= request_data->planned_path.size()){ // if the target cell has already been reserved and the reserving robot is closer to the target
        response_data->target_cell = target_cell; // adding target cell to response so robot knows which cell is already reserved by another robot
        cell_reserved = false; // return false as cell has not been reserved
    }
    else{ // if the current robot is closer to the target cell than the current reserving robot
        // reserving target cell + updating current planned path
        RobotInfo* robot_info = getRobotInfo(robot_id);// gathering planned_path for exploration
        robot_info->planned_path = request_data->planned_path; // setting planned path to robot's target
        robot_info->robot_target = target_cell; // gathering robot's target cell 

        // telling robot who was previously reserving the target cell to find a new target
        // reserving_robot->planned_path.clear(); // clearing planned path as a new target must be located
        reserving_robot->robot_target = NULL_COORDINATE; // setting robot target to an invalid value as no target is currently assigned

        updateRobotState(1, reserving_robot->Robot_Message_Reciever); // tell previous reserving robot to find a new target

        // response data loading
        cell_reserved = true; // cell has been reserved
    }

    response_data->cell_reserved = cell_reserved; // adding information is cell was reserved to response
   
    exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions); // adding request info to tracking JSON

    // attaching response data to message
    Message* response = new Message(t_Response, request->response_id); // creating new response with given response id

    RequestHandler* robot_request_handler = getTargetRequestHandler(robot_id); // getting request handler to send response

    if (robot_request_handler != NULL){ // if request handler gathered send data
        response->msg_data = response_data; // assigning response to message

        robot_request_handler->sendMessage(response); // sending message
    }
    else{ // if no request handler found, must delete response data
        delete response_data;
        delete response;
    }
    return;
}


RobotInfo* RobotMaster_IE::isCellReserved(Coordinates* target_cell, unsigned int robot_id){ // determines if another robot has already reserved the cell
    for(int i = 0; i < tracked_robots.size(); i++){
        if(tracked_robots[i].robot_target == *target_cell && tracked_robots[i].robot_id != robot_id){ // if the cell is currently a different robot's target
            return &(tracked_robots[i]);
        }
    }

    return NULL;
}