#include "MultiRobot_IE.h"

MultiRobot_IE::MultiRobot_IE(){

}

MultiRobot_IE::~MultiRobot_IE(){

}

void MultiRobot_IE::requestReserveCell(){
    
    transaction_counter++; // incrementing transaction counter as new request is being sent

    valid_responses.push_back(transaction_counter); // adding transaction to valid responses as response is required

    Message* temp_message = new Message(t_Request, transaction_counter); // generating message

    // gathering message data
    m_reserveCellRequest* message_data = new m_reserveCellRequest;  

    message_data->robot_id = id; // adding id of robot
    message_data->target_cell = planned_path[planned_path.size() - 1]; // adding target destination (end of planned path) of robot
     
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

void MultiRobot_IE::updateLocalMap(std::vector<Coordinates>* map_info, std::vector<std::vector<bool>>* edge_info, std::vector<char>* map_status){

    for(int i = 0; i < map_info->size(); i++){ // iterate through node information
        unsigned int x = (*map_info)[i].x; // gathering x and y position for data transfer
        unsigned int y = (*map_info)[i].y;

        LocalMap->nodes[y][x] = (*map_status)[i]; // passing map status of cell into LocalMap
        
        if((*map_status)[i] == 1){ // if the node is valid, pass various x and y edge information into LocalMap
            LocalMap->y_edges[y][x] = (*edge_info)[i][0]; // passing northern edge info into LocalMap
            LocalMap->y_edges[y + 1][x] = (*edge_info)[i][1]; // passing southern edge info into map
            LocalMap->x_edges[y][x] = (*edge_info)[i][2]; // passing eastern edge info into map
            LocalMap->x_edges[y][x + 1] = (*edge_info)[i][3]; // passing western edge info into map
        }
    }   
    
    return;
}

int MultiRobot_IE::handleCellReserveResponse(Message* response, int current_status){
        
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
                                // must update map with returned data so next closest cell can be reserved
            updateLocalMap(map_info, edge_info, map_status);

            new_robot_status = s_pathfind; // change status to pathfind as must try and reserve different with updated map info
        }
        else{  // if cell reserved
            new_robot_status = s_move_robot; // setting status to 3 so movement will occur on next loop cycle
        }
    }
    else{ // do nothing if other request type is sent
        new_robot_status = current_status;
    }

    return new_robot_status;
}