#include "MultiRobot_CellReservation.h"

MultiRobot_CellReservation::MultiRobot_CellReservation(){

}

MultiRobot_CellReservation::~MultiRobot_CellReservation(){

}

void MultiRobot_CellReservation::requestReserveCell(){
    
    transaction_counter++; // incrementing transaction counter as new request is being sent

    valid_responses.push_back(transaction_counter); // adding transaction to valid responses as response is required

    Message* temp_message = new Message(t_Request, transaction_counter); // generating message

    // gathering message data
    m_reserveCellRequest* message_data = new m_reserveCellRequest;  

    message_data->robot_id = id; // adding id of robot
    message_data->planned_path = planned_path; // adding planned path


    if(planned_path.size() > 1){ // if more than one element in planned_path, must pass second last element in deque as neigbouring cell
        message_data->neighbouring_cell = planned_path[planned_path.size() - 2]; // passing second last element in planned_path as neighbouring cell
    }
    else{ // as planned_path has only 1 element, cell must be neighbouring current position thus pass current position as neighbouring cell
        Coordinates c(x_position,y_position); // gathering current robot coordinates
        message_data->neighbouring_cell = c; // passing current robot position as neighbouring cell
    }


    // attaching message data
    temp_message->msg_data = message_data;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    return;
}


int MultiRobot_CellReservation::handleCellReserveResponse(Message* response, int current_status){
        
    // gathering response type for switch statement
    m_genericRequest* response_data = (m_genericRequest*) response->msg_data; // use generic message pointer to gather request type

    int new_robot_status;

    if(response_data->request_type == reserveCellRequest_ID){ // if cell reservation request is being handled

        m_reserveCellResponse* message_response = (m_reserveCellResponse*)response_data;  // typecasting response data to appropriate format
        
        bool reserved_succeed = message_response->cell_reserved; // gather whether cell has been reserved

        std::vector<Coordinates>* map_info = message_response->map_coordinates; // gather node map for map update
        std::vector<std::vector<bool>>* edge_info = message_response->map_connections; // gather wall information for each node in map
        std::vector<char>* map_status = message_response->map_status; // gather status of nodes tracked in returned map

        if(!reserved_succeed){ // if failed to reserve cell found by pathfinding
                                
            if(map_info->size() == 0){ // if a map update is not required (no nodes added to map_info), the target cell if already reserved by another robot

                already_reserved_cells.push_back(message_response->target_cell); // must add location to already reserved cells to ensure it is not reserved again
            }
            else{
                // must update map with returned data so next closest cell can be reserved
                updateLocalMap(map_info, edge_info, map_status);
            }

            new_robot_status = s_pathfind; // change status to pathfind as must try and reserve different with updated map info
        }
        else{  // if cell reserved
            already_reserved_cells.clear(); // can clear already reserved cells 
            new_robot_status = s_move_robot; // setting status to 3 so movement will occur on next loop cycle
        }
    }
    else{ // do nothing if other request type is sent
        new_robot_status = current_status;
    }

    return new_robot_status;
}

bool MultiRobot_CellReservation::isCellAlreadyReserved(Coordinates *C){ // checks if a cell is already reserved by another robot

    for(int i = 0; i < already_reserved_cells.size(); i++){
        if(already_reserved_cells[i] == *C){ // if cell is already reserved
            return true;
        }
    }

    return false; // if cell is not already reserved
}

bool MultiRobot_CellReservation::BFS_exitCondition(Coordinates* node_to_test){ // overriden pathfinding exit condition
    return (Robot::BFS_exitCondition(node_to_test) && !isCellAlreadyReserved(node_to_test)); // return true if cell is not reserved and is unexplored
}

void MultiRobot_CellReservation::BFS_noPathFound(){
    already_reserved_cells.clear(); // clearing already reserved cells as no unreserved cell found
                                    // these cells should be queried again by robot

    return;
}