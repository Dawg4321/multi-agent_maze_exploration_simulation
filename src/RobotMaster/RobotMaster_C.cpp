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
        CollisionMatrix[target_cell.y][target_cell.x].occupying_robot == robot_id; // reserving selected cell to prevent collision
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

void RobotMaster_C::updateRobotLocation(unsigned int* id, Coordinates* new_cell){ // updates the location of a robot to the location specified

    for(int i = 0; i < tracked_robots.size(); i++){ // finding robot to update
        if (tracked_robots[i].robot_id == *id){ // if robot found using id
            CollisionMatrix[tracked_robots[i].robot_position.y][tracked_robots[i].robot_position.x].occupying_robot = 0; // unreserving cell previous location as movement has been completed
            tracked_robots[i].robot_position = *new_cell; // updating position in RobotInfo
            break; 
        }
    }

    return;
}

unsigned int RobotMaster_C::addRobot(unsigned int x, unsigned int y, RequestHandler* r){ // adding robot to control system
                                                                                       // this must be completed by all robots before beginning exploration

    unsigned int id = RobotMaster::addRobot(x, y, r); // calling parent add robot function to get new id

    CollisionMatrix[y][x].occupying_robot = id; // adding current cell to robot occupation list

    return id;
}