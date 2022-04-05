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

int MultiRobot_C::handleCollisionResponse(Message* response, int current_status){ // handles a collision message response
                                                                                  // This is meant to be used in conjuction with RobotMaster::handleMasterResponse()
                                                                                  // status is only updated if collision request is received

    // gathering response type for switch statement
    m_genericRequest* response_data = (m_genericRequest*) response->msg_data; // use generic message pointer to gather request type

    int new_robot_status;

    if(response_data->request_type == move2CellRequest_ID){
        
        m_move2CellResponse* message_response = (m_move2CellResponse*)response_data;

        bool movement_can_occur = message_response->can_movement_occur; // gather whether movement can occur

        if(movement_can_occur){
            new_robot_status = s_move_robot;
        }
        else{
            new_robot_status = s_pathfind;
        }
    }
    else{ // if the request type is not a collision response, stay is currently selected status
        new_robot_status = current_status;
    }
    return new_robot_status;
}