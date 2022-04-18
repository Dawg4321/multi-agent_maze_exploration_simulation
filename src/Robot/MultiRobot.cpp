#include "MultiRobot.h"

MultiRobot::MultiRobot(int x, int y, RequestHandler* outgoing_req, unsigned int xsize, unsigned int ysize): Robot(x, y, xsize, ysize){

    Robot_2_Master_Message_Handler = outgoing_req; // assigning message handler for robot -> master communications
    Master_2_Robot_Message_Handler = new RequestHandler; // assigning message handler for master -> robot communications

    accepting_requests = false; // robot needs to be added to RobotMaster before it can accept requests

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

        last_request_priority = 10; // reseting last_request priority to a high value
                                    // if a received RobotMaster request has lower priority than this value
                                    // set the request priority so only request of same priority value or lower are handled 
        
        do{ // repeatedly process all messages until there are none left

            if(request->message_type){ // if sent message is a response for a previously sent request

                // need to check if response is stale (e.g. another request has been sent overriding the previous one)
                if(!(isResponseStale(request->response_id))){ // if response is not stale (transaction counter has not been changed since request sent)
                    
                    new_robot_status = handleMasterResponse(request, current_status); // call Response handling function

                    makeResponseStale(request->response_id); // response can now be made stale as it has been handled
                }
                else{ // if response is stale (another request has already been sent)
                    // keep previously set status
                    new_robot_status = current_status;
                }
            }
            else{ // if message is a request from master
                if(accepting_requests){ // if the robot is accepting requests, handle them
                    new_robot_status = handleMasterRequest(request, current_status);
                }
            }

            // message has been handled, can delete sent data
            delete request->msg_data; // deleting data receieved in the request as its no longer needed
            delete request; // deleting recieved message as no longer needed

            request = Master_2_Robot_Message_Handler->getMessage(); // gather next request to process
            current_status = new_robot_status; // setting new robot status as current status for next iteration through for loop

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
            if(last_request_priority >= 1){ // if the current request priority allows for this request to be handled
                
                // updating robot state to specified value
                m_updateRobotStateRequest* data = (m_updateRobotStateRequest*) request->msg_data;
                new_robot_status = data->target_state;
                
                valid_responses.clear(); // clearing valid_responses to cause any outstanding responses to become stale
                                         // this is done as the update state message overrides any responses after it

                if(new_robot_status == s_shut_down){ // if robot has been told to shut down
                    last_request_priority = 0; // set request priority to a value where no future requests will be computed and prevent shutdown
                }
                else{ // if robot has not been told to shutdown
                    last_request_priority = 1; // set priority so lower priority requests are not handled
                }
            }
            else{ // if the request priority is a value which does not allow update robot state request to be handled
                
            }

            break;
        }
        default:
        {
            new_robot_status = current_status; // keep current status if an undefined status is received

            break;
        }
    }
    
    return new_robot_status; // returning changes to robot status
}

int MultiRobot::handleMasterResponse(Message* response, int current_status){
    // gathering response type for switch statement
    m_genericRequest* response_data = (m_genericRequest*) response->msg_data; // use generic message pointer to gather request type

    int new_robot_status; // status to be returned

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

            accepting_requests = true; // robot master has been added to the system meaning it can now accept requests from the RobotMaster

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
        default: // TODO: Implement handling if improper messge type is recieved
        {
            new_robot_status = current_status; // keep current status if improper response is received

            break;
        }
    }
    
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

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    return;
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
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    return;
}

void MultiRobot::updateLocalMap(std::vector<Coordinates>* map_info, std::vector<std::vector<bool>>* edge_info, std::vector<char>* map_status){

    for(int i = 0; i < map_info->size(); i++){ // iterate through node information
        int x = (*map_info)[i].x; // gathering x and y position for data transfer
        int y = (*map_info)[i].y;

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

void MultiRobot::computeScanCell(GridGraph* maze){ // function used to compute the cell scanning funciton of the robot
    
    std::vector<bool> connection_data = scanCell(maze); // scan cell which is occupied by the robot 

    requestGlobalMapUpdate(connection_data); // sending message to master with scanned maze information

    robot_status = s_pathfind; // setting status to 2 so pathfinding will occur on next loop cycle

    return;
}

void MultiRobot::computeMove(){ // function used to compute movement of the robot

    bool move_occured = move2Cell(planned_path[0]); // attempt to move robot to next location in planned path queue

    if(move_occured){ // if movement succeed 
        planned_path.pop_front(); // remove element at start of planned path queue as it has occured 
    
        if(planned_path.empty()){ // if there are no more moves to occur, must be at an unscanned cell
            robot_status = s_scan_cell; // set robot to scan cell on next loop iteration as at desination cell
        }
        else{ // if more moves left, try another movement
            robot_status = s_move_robot;
        }
    }
    else{ // if movement failed
            planned_path.clear(); // clearing current planned path
            robot_status = s_pathfind; // attempt to plan a new path which will hopefully not cause movement to faile
    }

    return;
}