#include "MultiRobot_C.h"

MultiRobot_C::MultiRobot_C(){

}

MultiRobot_C::~MultiRobot_C(){

}

void MultiRobot_C::requestMove2Cell(Coordinates target_cell){
    // need to message controller to see if target cell is occupied
    // if unoccupied, robot can move to cell
    // if cell is unoccupied (e.g. robot can move to cell), return true
    // if cell is occupied, return false
    // these are handled in response

    transaction_counter++; // incrementing transaction counter as new request is being sent

    valid_responses.push_back(transaction_counter); // adding transaction to valid responses as response is required

    Message* temp_message = new Message(t_Request, transaction_counter);

    // defining new Move2CellRequest
    m_move2CellRequest* message_data = new m_move2CellRequest;

    // attaching message data to request
    message_data->robot_id = id; // adding id of robot
    message_data->target_cell = target_cell; // adding target destination of robot
    temp_message->msg_data = message_data;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller
    
    return;
}

void MultiRobot_C::requestGetMap(){
    
    transaction_counter++;

    valid_responses.push_back(transaction_counter); // adding transaction to valid responses as response is required

    Message* temp_message = new Message(t_Request, transaction_counter);

    m_getMapRequest* message_data = new m_getMapRequest;

    message_data->robot_id = id; // adding id of robot
    message_data->current_cell = Coordinates(x_position, y_position); // adding current position
    message_data->target_cell = target_2_pathfind; // adding target cell 

    temp_message->msg_data = message_data;
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    return;
}

int MultiRobot_C::handleCollisionResponse(Message* response, int current_status){ // handles a collision message response
                                                                                  // This is meant to be used in conjuction with RobotMaster::handleMasterResponse()
                                                                                  // status is only updated if collision request is received

    // gathering response type for switch statement
    m_genericRequest* response_data = (m_genericRequest*) response->msg_data; // use generic message pointer to gather request type

    int new_robot_status;

    switch(response_data->request_type){
        case move2CellRequest_ID:
        {
            m_move2CellResponse* message_response = (m_move2CellResponse*)response_data;
            bool movement_can_occur = message_response->can_movement_occur; // gather whether movement can occur

            if(movement_can_occur){ // if movement can occur, move to next cell in planned_path
                new_robot_status = s_compute_move;
            }
            else{ // if movement cannot occur, create new planned_path
                new_robot_status = s_pathfind;
            }

            break;
        }
        case getMapRequest_ID:
        {
            m_getMapResponse* message_response = (m_getMapResponse*)response_data;

            updateLocalMap(&message_response->map_coordinates, &message_response->map_connections, &message_response->map_status); // updating local map with recieved information

            new_robot_status = s_pathfind2target; // attempt to pathfind to received target again

            break;
        }
        default:
        { // if the request type is not a collision response, do nothin (stay is currently selected status)
            new_robot_status = current_status;
            break;
        }
    }

    return new_robot_status;
}

int MultiRobot_C::handleCollisionRequest(Message* request, int current_status){

    // gathering request type for switch statement
    m_genericRequest* r = (m_genericRequest*) request->msg_data; // use generic message pointer to gather request type

    int new_robot_status;

    switch (r->request_type){ // determining type of request before processing

        case setTargetCell_ID: // setTargetCell
        {   
            // updating robot state to specified value
            m_setTargetCellRequest* data = (m_setTargetCellRequest*) request->msg_data;
            target_2_pathfind = data->new_target_cell; // passing new target onto robot

            planned_path.clear(); // clearing planned path as robot needs to create a new path to the new target

            valid_responses.clear(); // clearing valid_responses to cause any outstanding responses to become stale
                                     // this is done as setTargetCell message overrides any responses after it

            new_robot_status = s_pathfind2target; // attempt to pathfind to target with new target

            break;
        }
        default:
        {
            new_robot_status = current_status; // keep current status if improper request is received

            break;
        }
    }
    
    return new_robot_status; // retshared_ptr
}