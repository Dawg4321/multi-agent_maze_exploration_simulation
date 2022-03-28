#include "MultiRobot.h"

MultiRobot::MultiRobot(unsigned int x, unsigned int y, RequestHandler* outgoing_req, unsigned int xsize, unsigned int ysize): Robot(x, y, xsize, ysize){

    Robot_2_Master_Message_Handler = outgoing_req; // assigning message handler for robot -> master communications
    Master_2_Robot_Message_Handler = new RequestHandler; // assigning message handler for master -> robot communications

    response_sem = new sem_t; // semaphore to signal from controller to robot that the response message is ready 
    sem_init(response_sem, 0, 0); // initializing semaphore to value of 0. this will cause robot to wait until response is ready
                                  
    acknowledgement_sem = new sem_t; // semaphore to signal from robot to controller that the current response message has been analysed  
    sem_init(acknowledgement_sem, 0, 0); // initializing semaphore to value of 0. this will cause controller to wait until robot is done with response

    transaction_counter = 0; // initializing to 0 as no transactions have occured
}

MultiRobot::~MultiRobot(){
    delete Master_2_Robot_Message_Handler; // deleting Master_2_Robot_Message_Handler as no longer needed as object is being destroyed
}

bool MultiRobot::move2Cell(Coordinates destination){ // overriden version of move2Cell using destination
                                                      // converts target destination to a direction then uses original move2Cell function
                                                      // messages master to confirm wh

    // bool is_not_occupied = requestMove2Cell(destination); // checking if target cell is unoccupied

    //if(is_not_occupied){ // attempt to move as cell unoccupied

    bool movement_occurred = Robot::move2Cell(destination); // moving robot using Robot classes move function
    
    requestRobotLocationUpdate(); // notifying master of robot position update

    return movement_occurred; // returning whether movement occured successfully
    /*}
    else {
        return false; // return false as target cell occupied
    }*/
    
}


int MultiRobot::getMessagesFromMaster(int current_status){ // checking if RobotMaster wants robot to change states
                                                           // this should only be called if robot is not waiting for a response

    Message* request = Master_2_Robot_Message_Handler->getMessage(); // gathering request from msg_queue
                                                                     // pointer is gathered so response can be gathered by robot threads
                                                                     // returned pointer will be NULL is no messages to get
    
    int new_robot_status; // return variable to update status in RobotLoop

    if(request != NULL){ // if there is a a request to handle, process it
        
        do{ // repeatedly process all messages until there are none left

            if(request->message_type){ // if sent message is a response for a previously sent request

                // need to check if response is stale (e.g. another request has been sent overriding the previous one)
                if(!(isResponseStale(request->response_id))){ // if response is not stale (transaction counter has not been changed since request sent)
                    
                    new_robot_status = handleMasterResponse(request, current_status); // call Response handling function

                }
                else{ // if response is stale (another request has already been sent)
                    // do nothing
                }
            }
            else{ // if message is a request from master
                new_robot_status = handleMasterRequest(request, current_status);
            }

            // message has been handled, can delete sent data
            delete request->msg_data; // deleting data receieved in the request as its no longer needed
            delete request; // deleting recieved message as no longer needed

            request = Master_2_Robot_Message_Handler->getMessage(); // gather next request to process

        } while(request != NULL); // checking if there is another request to process
    }
    else{ // if no message to handle, do nothing and keep current status
        new_robot_status = current_status; // set new robot status status as current status
    }
    
    return new_robot_status;
}

int MultiRobot::handleMasterRequest(Message* request, int current_status){
    // gathering request type for switch statement
    m_genericRequest* r = (m_genericRequest*) request->msg_data; // use generic message pointer to gather request type

    int new_robot_status;

    switch (r->request_type){ // determining type of request before processing

        case updateRobotStateRequest_ID: // update robot state
        {   
            // updating robot state to specified value
            m_updateRobotStateRequest* data = (m_updateRobotStateRequest*) request->msg_data;
            new_robot_status = data->target_state;

            valid_responses.clear(); // clearing valid_responses to cause any outstanding responses to become stale
                                     // this is done as the update state message overrides any responses after it

            break;
        }
        default:
        {
            new_robot_status = current_status; // keep current status if improper request is received

            break;
        }
    }
    
    return new_robot_status; // returning changes to robot status
}

int MultiRobot::handleMasterResponse(Message* response, int current_status){
    // gathering response type for switch statement
    m_genericRequest* response_data = (m_genericRequest*) response->msg_data; // use generic message pointer to gather request type

    int new_robot_status;

    switch (response_data->request_type){ // determining type of response to process
        case shutDownRequest_ID:
        {
            // final response from master received, can exit loop now       
            new_robot_status = s_exit_loop; // setting status to exit loop to cause a loop exit
            
            break;
        }
        case addRobotRequest_ID: // addRobot response 
        {   
            // This response type assigns a new id to the robot

            m_addRobotResponse* message_response = (m_addRobotResponse*)response_data; // typecasting response data to appropriate format
            
            id = message_response->robot_id; // assigning id to Robot from RobotMaster's response

            new_robot_status = current_status; // keep current robot status as master has not told robot to begin exploring
            
            break;
        }
        case updateGlobalMapRequest_ID: // updateGlobalMapRequest response 
        {
            // Empty as currently no response needed
            // responses will still be recieved to allow for future implementation if needed

            new_robot_status = current_status; // status does not change as this response type is not implemented
            
            break;
        }
        case updateRobotLocationRequest_ID:
        {
            // Empty as currently no response needed
            // responses will still be recieved to allow for future implementation if needed

            new_robot_status = current_status; // status does not change as this response type is not implemented

            break;
        }
        case move2CellRequest_ID:{
            // do nothing right now **
            // TODO: implement move2 cell response
            new_robot_status = current_status;

            break;
        }
        case reserveCellRequest_ID: 
        {
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

            break;
        }
        default: // TODO: Implement handling if improper messge type is recieved
        {
            new_robot_status = current_status; // keep current status if improper response is received

            break;
        }
    }

    makeResponseStale(response->response_id); // response can now be made stale as it has been handled
    
    return new_robot_status; // returning changes to robot status
}

bool MultiRobot::isResponseStale(int transaction_id){ // return false if given id is in the valid responses vector (e.g. it has not been made stale)
    
    for(int i = 0; i < valid_responses.size(); i++){ 
        if(valid_responses[i] == transaction_id){
            return false; // given id is in vector thus not stale -> return false
        }
    }
    return true; // // given id is not in vector thus stale -> return true
}

void MultiRobot::makeResponseStale(int transaction_id){ // remove transaction from valid_responses
    
    for(int i = 0; i < valid_responses.size(); i++){ // iterate through non-stale responses
        if(valid_responses[i] == transaction_id){ // if transaction in array, erase it
            valid_responses.erase(valid_responses.begin() + i);
            break;
        }
    }

    return;
}

void MultiRobot::assignIdFromMaster(){

    transaction_counter++; // incrementing transaction counter as new request is being sent

    valid_responses.push_back(transaction_counter); // adding transaction to valid responses as response is required

    Message* temp_message = new Message(t_Request, transaction_counter); // buffer to load data into before sending message

    // defining new addRobotRequest
    m_addRobotRequest* message_data = new m_addRobotRequest;
    
    // gather request data
    message_data->x = x_position; // adding x position of robot
    message_data->y = y_position; // adding y position of robot 
    message_data->robot_request_handler =  Master_2_Robot_Message_Handler; // adding request handler for master -> robot message to [2]

    // attaching message data to request
    temp_message->msg_data = message_data;

    // attaching robot communication semaphores to message
    temp_message->res_sem = response_sem; 
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    return;
}

bool MultiRobot::requestMove2Cell(Coordinates target_cell){
    // need to message controller to see if target cell is occupied
    // if unoccupied, robot can move to cell
    // if cell is unoccupied (e.g. robot can move to cell), return true
    // if cell is occupied, return false

    transaction_counter++; // incrementing transaction counter as new request is being sent

    Message* temp_message = new Message(t_Request, transaction_counter);

    // defining new Move2CellRequest
    m_move2CellRequest* message_data = new m_move2CellRequest;

    // attaching message data to request
    message_data->robot_id = id; // adding id of robot
    message_data->target_cell = target_cell; // adding target destination of robot
    temp_message->msg_data = message_data;

    // attaching robot communication semaphores to message
    temp_message->res_sem = response_sem; 
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    sem_wait(response_sem); // waiting for response to be ready from controller

    m_move2CellResponse* message_response = (m_move2CellResponse*)temp_message->return_data;

    bool can_movement_occur = message_response->can_movement_occur; // gather whether movement can occur
    
    sem_post(acknowledgement_sem); // signalling to controller that the response has been utilised 
    
    return can_movement_occur;
}

void MultiRobot::requestShutDown(){
    // sending message to notify RobotMaster that robot is ready to shutdown

    transaction_counter++; // incrementing transaction counter as new request is being sent

    valid_responses.push_back(transaction_counter); // adding transaction to valid responses as response is required

    Message* temp_message = new Message(t_Request, transaction_counter); // buffer to load data into before sending message

    // defining new shutDownRequest
    m_shutDownRequest* message_data = new m_shutDownRequest;

    // attaching message data to request
    message_data->robot_id = id; // adding id of robot sending request
    temp_message->msg_data = message_data;
    
    temp_message->res_sem = response_sem; // attaching communication semaphores to message
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    return;
}

void MultiRobot::requestRobotLocationUpdate(){

    transaction_counter++; // incrementing transaction counter as new request is being sent

    // sending message with new robot position
    Message* temp_message = new Message(t_Request, transaction_counter); // buffer to load data into before sending message

    // attaching message data to request
    m_updateRobotLocationRequest* message_data = new m_updateRobotLocationRequest;    
    message_data->robot_id = id; // adding id of robot sending request
    Coordinates robot_cords(x_position,y_position);// gathering robots current coordinates
    message_data->new_robot_location = robot_cords; // adding robot location

    temp_message->msg_data = message_data;

    // attaching communication semaphores to message
    temp_message->res_sem = response_sem; 
    temp_message->ack_sem = acknowledgement_sem;
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    return;
}

void MultiRobot::requestGlobalMapUpdate(std::vector<bool> connection_data){
    
    transaction_counter++; // incrementing transaction counter as new request is being sent

    // sending message with scanned maze information
    Message* temp_message = new Message(t_Request, transaction_counter); // buffer to load data into before sending message

    // attaching message data to request
    m_updateGlobalMapRequest* message_data = new m_updateGlobalMapRequest;  
    message_data->robot_id = id; // adding id of robot sending request
    message_data->wall_info = connection_data; // adding vector containing information on walls surrounding robot to [1]
    Coordinates robot_cords(x_position,y_position);// gathering robots current coordinates
    message_data->cords = robot_cords; // current coordinates of where the read occured

    temp_message->msg_data = message_data;

    // attaching communication semaphores to message
    temp_message->res_sem = response_sem; 
    temp_message->ack_sem = acknowledgement_sem;
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    return;
}

void MultiRobot::updateLocalMap(std::vector<Coordinates>* map_info, std::vector<std::vector<bool>>* edge_info, std::vector<char>* map_status){

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

void MultiRobot::requestReserveCell(){
    
    transaction_counter++; // incrementing transaction counter as new request is being sent

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

    temp_message->res_sem = response_sem; // attaching robot communication semaphores to message
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    sem_wait(response_sem); // waiting for response to be ready from controller

    m_reserveCellResponse* message_response = (m_reserveCellResponse*) temp_message->return_data;

    return;
}