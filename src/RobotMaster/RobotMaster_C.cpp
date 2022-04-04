#include "RobotMaster_C.h"

RobotMaster_C::RobotMaster_C(unsigned int xsize, unsigned int ysize){
    CollisionMatrix.resize(ysize,std::vector<CollisionInfo>(xsize)); // resizing ReservationMatrix info to maze size
}

RobotMaster_C::~RobotMaster_C(){

}

void RobotMaster_C::move2CellRequest(Message* request){

    // gathering incoming request
    m_move2CellRequest* request_data = (m_move2CellRequest*)request->msg_data; 
    
    unsigned int robot_id = request_data->robot_id;
    Coordinates target_cell = request_data->target_cell;

    m_move2CellResponse* response_data = new m_move2CellResponse;

    if(CollisionMatrix[target_cell.y][target_cell.x].occupying_robot == 0){ // if target cell is unoccupied
        response_data->can_movement_occur = true; // movement can occur
    }
    else if(CollisionMatrix[target_cell.y][target_cell.x].occupying_robot == robot_id){ // if the occupying robot is the current robot
        printf("Critical Error: Robot attempting to move to a cell it already occupies\n");
        response_data->can_movement_occur = false; // movement can't occur
    }
    else{ // target cell is occupied by another robot
        response_data->can_movement_occur = false; // movement can't occur
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