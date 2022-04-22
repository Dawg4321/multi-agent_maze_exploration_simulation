#include "RobotMaster_CellReservation.h"

RobotMaster_CellReservation::RobotMaster_CellReservation(){

}

RobotMaster_CellReservation::~RobotMaster_CellReservation(){

}

void RobotMaster_CellReservation::reserveCellRequest(Message* request){
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

    RobotInfo* robot_info = getRobotInfo(robot_id); // gathering robot who wants to reserve the cell
    
    robot_info->planned_path.clear(); // clearing planned path and robot moving flag in the event that these are set despite robot not moving
    robot_info->robot_moving = false; // these may not be set if robot has sent a message saying he will move but then receives a request to find a new target

    // return data allocation
    m_reserveCellResponse* response_data = new m_reserveCellResponse; // response message

    bool cell_reserved; // variable to whether cell reservation is possible

    RobotInfo* reserving_robot = isCellReserved(&target_cell, robot_id); // checking if another robot is reserving the target cell

    // processing if cell can be reserved
    // in this case, vectors allocated in response_data will be modified
    if(GlobalMap->nodes[target_cell.y][target_cell.x] == 1){ // if the target cell has already been explored
        // gathering portion of map outwards from unexplored node to return to robot inorder to expand its LocalMap
        gatherPortionofMap(target_cell, neighbouring_cell, response_data->map_coordinates, response_data->map_connections, response_data->map_status);
        
        response_data->cell_reserved = false; // adding information about cell not being reserved to response
    }
    else if(reserving_robot == NULL){ // no other robot has reserved the target cell and it has not been explored
        
        reserveCell(robot_info, &request_data->planned_path, target_cell); // reserving target cell + updating current planned path

        response_data->cell_reserved = true; // adding information about cell being reserved to response
    }
    else{ // if two robots are trying to reserve the same cell
        handleAlreadyReservedCell(robot_info, reserving_robot, request_data, response_data, target_cell); // handle this using an approach specified in a child class
    }
   
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

void RobotMaster_CellReservation::reserveCell(RobotInfo* robot_info, std::deque<Coordinates>* path_2_target , Coordinates target_cell){
    
    robot_info->planned_path = *path_2_target; // setting planned path of robot
    robot_info->robot_target = target_cell; // setting new target cell in robot's robot_info

    return;
}


RobotInfo* RobotMaster_CellReservation::isCellReserved(Coordinates* target_cell, unsigned int robot_id){ // determines if another robot has already reserved the cell
    for(int i = 0; i < tracked_robots.size(); i++){
        if(tracked_robots[i].robot_target == *target_cell && tracked_robots[i].robot_id != robot_id){ // if the cell is currently a different robot's target
            return &(tracked_robots[i]);
        }
    }

    return NULL;
}