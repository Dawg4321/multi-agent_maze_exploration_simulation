#include "RobotMaster_IE.h"

RobotMaster_IE::RobotMaster_IE(unsigned int xsize, unsigned int ysize){
    ReservationMatrix.resize(ysize,std::vector<ReservationInfo>(xsize)); // resizing ReservationMatrix info to maze size
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
    Coordinates target_cell = request_data->target_cell;                    
    Coordinates neighbouring_cell = request_data->neighbouring_cell;

    // return data allocation
    m_reserveCellResponse* response_data = new m_reserveCellResponse; // response message

    bool cell_reserved; // variable to whether cell reservation is possible

    // processing if cell can be reserved
    // in this case, vectors allocated in response_data will be modified
    if(GlobalMap->nodes[target_cell.y][target_cell.x] == 1){ // if the target cell has already been explored
        // gathering portion of map outwards from unexplored node to return to robot
        gatherPortionofMap(target_cell, neighbouring_cell, response_data->map_coordinates, response_data->map_connections, response_data->map_status);
        cell_reserved = false; // cell has not been successfully reserved for requesting robot
    }
    else if (ReservationMatrix[target_cell.y][target_cell.x].reserved > 0){ // if the target cell has already been reserved
        response_data->target_cell = target_cell; // adding target cell to response so robot knows that cell is already reserved by another robot
        cell_reserved = false; // return false as cell has not been reserved
    }
    else{ // if unreserved and unexplored
        ReservationMatrix[target_cell.y][target_cell.x].reserved = robot_id; // reserving cell for exploration
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

void RobotMaster_IE::unreserveCell(Coordinates* C){ // unreserved cell at selected location
    ReservationMatrix[C->y][C->x].reserved = 0; // setting to 0 as no robot has id of 0

    return;
}

void RobotMaster_IE::updateGlobalMap(unsigned int* id, std::vector<bool>* connections, Coordinates* C){
    
    RobotMaster_IE::unreserveCell(C); // unreserving cell before updating GlobalMap as it has been scanned

    RobotMaster::updateGlobalMap(id, connections, C); // updating GlobalMap
}