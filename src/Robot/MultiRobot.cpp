#include "MultiRobot.h"

MultiRobot::MultiRobot(unsigned int x, unsigned int y, RequestHandler* outgoing_req, unsigned int xsize, unsigned int ysize): Robot(x, y, xsize, ysize){

    Robot_2_Master_Message_Handler = outgoing_req; // assigning message handler for robot -> master communications
    Master_2_Robot_Message_Handler = new RequestHandler; // assigning message handler for master -> robot communications

    response_sem = new sem_t; // semaphore to signal from controller to robot that the response message is ready 
    sem_init(response_sem, 0, 0); // initializing semaphore to value of 0. this will cause robot to wait until response is ready
                                  
    acknowledgement_sem = new sem_t; // semaphore to signal from robot to controller that the current response message has been analysed  
    sem_init(acknowledgement_sem, 0, 0); // initializing semaphore to value of 0. this will cause controller to wait until robot is done with response
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


int MultiRobot::getRequestsFromMaster(int status){ // checking if RobotMaster wants robot to change states
    
    Message* request = Master_2_Robot_Message_Handler->getMessage(); // gathering request from msg_queue
                                                                     // pointer is gathered so response can be gathered by robot threads
                                                                     // returned pointer will be NULL is no messages to get
    
    int ret_variable; // return variable to update status in RobotLoop

    if(request != NULL){ // if there is a a request to handle, process it

        // gathering request type for switch statement
        m_genericRequest* r = (m_genericRequest*) request->msg_data; // use generic message pointer to gather request type

        switch (r->request_type){ // determining type of request before processing

                case updateRobotStateRequest_ID: // update robot state
                {   
                    // updating robot state to specified value
                    m_updateRobotStateRequest* data = (m_updateRobotStateRequest*) request->msg_data;
                    ret_variable = data->target_state;
                    
                    delete data; // deleting data receieved in the request as its no longer needed
                    delete request; // deleting recieved message as no longer needed
                    
                    break;
                }
                default: // TODO: Implement handling if improper messge type is recieved
                {
                    break;
                }
            }
    }
    else{ // if no message to handle, do nothing
        ret_variable = status; // set status as current status
    }
    
    return ret_variable;
}

void MultiRobot::assignIdFromMaster(){

    // sending addRobot request to RobotMaster to get ID
    Message* temp_message = new Message(); // buffer to load data into before sending message

    // defining new addRobotRequest
    m_addRobotRequest* message_data = new m_addRobotRequest;
    
    // gather request data
    message_data->x = x_position; // adding x position of robot
    message_data->y = y_position; // adding y position of robot 
    message_data->robot_request_handler =  Master_2_Robot_Message_Handler; // adding request handler for master -> robot message to [2]

    // attaching message data to request
    temp_message->msg_data = (void*) message_data;

    // attaching robot communication semaphores to message
    temp_message->res_sem = response_sem; 
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    sem_wait(response_sem); // waiting for response to be ready from controller

    m_addRobotResponse* message_response = (m_addRobotResponse*)temp_message->return_data; // gathering response data
    id = message_response->robot_id; // assigning id from RobotMaster's response
    
    sem_post(acknowledgement_sem); // signalling to controller that the response has been utilised 

    return;
}

bool MultiRobot::requestMove2Cell(Coordinates target_cell){
    // need to message controller to see if target cell is occupied
    // if unoccupied, robot can move to cell
    // if cell is unoccupied (e.g. robot can move to cell), return true
    // if cell is occupied, return false
    Message* temp_message = new Message;

    // defining new Move2CellRequest
    m_move2CellRequest* message_data = new m_move2CellRequest;

    // attaching message data to request
    message_data->robot_id = id; // adding id of robot
    message_data->target_cell = target_cell; // adding target destination of robot
    temp_message->msg_data = (void*) temp_message;

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
    // sending message to notidy RobotMaster that robot is ready to shutdown
    Message* temp_message = new Message(); // buffer to load data into before sending message

    // defining new shutDownRequest
    m_shutDownRequest* message_data = new m_shutDownRequest;

    // attaching message data to request
    message_data->robot_id = id; // adding id of robot sending request
    temp_message->msg_data = (void*) message_data;
    
    temp_message->res_sem = response_sem; // attaching communication semaphores to message
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    sem_wait(response_sem); // waiting for Master to acknowledge shutdown

    return;
}

void MultiRobot::requestRobotLocationUpdate(){
    // sending message with new robot position
    Message* temp_message = new Message(); // buffer to load data into before sending message

    // attaching message data to request
    m_updateRobotLocationRequest* message_data = new m_updateRobotLocationRequest;    
    message_data->robot_id = id; // adding id of robot sending request
    Coordinates robot_cords(x_position,y_position);// gathering robots current coordinates
    message_data->new_robot_location = robot_cords; // adding robot location

    temp_message->msg_data = (void*) message_data;

    // attaching communication semaphores to message
    temp_message->res_sem = response_sem; 
    temp_message->ack_sem = acknowledgement_sem;
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    sem_wait(response_sem); // waiting for data to be inputted into Master before continuing
    sem_post(acknowledgement_sem);

    return;
}

void MultiRobot::requestGlobalMapUpdate(std::vector<bool> connection_data){
    // sending message with scanned maze information
    Message* temp_message = new Message(); // buffer to load data into before sending message

    // attaching message data to request
    m_updateGlobalMapRequest* message_data = new m_updateGlobalMapRequest;  
    message_data->robot_id = id; // adding id of robot sending request
    message_data->wall_info = connection_data; // adding vector containing information on walls surrounding robot to [1]
    Coordinates robot_cords(x_position,y_position);// gathering robots current coordinates
    message_data->cords = robot_cords; // current coordinates of where the read occured

    temp_message->msg_data = (void*) message_data;

    // attaching communication semaphores to message
    temp_message->res_sem = response_sem; 
    temp_message->ack_sem = acknowledgement_sem;
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    sem_wait(response_sem); // waiting for data to be inputted into Master before continuing
    sem_post(acknowledgement_sem);

    return;
}

void MultiRobot::updateLocalMap(std::vector<Coordinates>* map_info, std::vector<std::vector<bool>>* edge_info, std::vector<char>* map_status){

    for(int i = 0; i < map_info->size(); i++){ // iterate through node information
        unsigned int x = (*map_info)[i].x; // gathering x and y position for data transfer
        unsigned int y = (*map_info)[i].y;

        LocalMap->nodes[y][x] = (*map_status)[i]; // passing map status of cell into LocalMap
        
        LocalMap->y_edges[y][x] = (*edge_info)[i][0]; // passing northern edge info into LocalMap
        LocalMap->y_edges[y + 1][x] = (*edge_info)[i][1]; // passing southern edge info into map
        LocalMap->x_edges[y][x] = (*edge_info)[i][2]; // passing eastern edge info into map
        LocalMap->x_edges[y][x + 1] = (*edge_info)[i][2]; // passing western edge info into map
    }   
    
    return;
}

bool MultiRobot::requestReserveCell(){
    
    Message* temp_message = new Message; // generating message

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
    temp_message->msg_data = (void*) message_data;

    temp_message->res_sem = response_sem; // attaching robot communication semaphores to message
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    sem_wait(response_sem); // waiting for response to be ready from controller

    m_reserveCellResponse* message_response = (m_reserveCellResponse*) temp_message->return_data;
    bool reserved_succeed = message_response->cell_reserved; // gather whether cell has been reserved
    
    std::vector<Coordinates>* map_info = message_response->map_coordinates; // gather node map for map update
    std::vector<std::vector<bool>>* edge_info = message_response->map_connections; // gather wall information for each node in map
    std::vector<char>* map_status = message_response->map_status; // gather status of nodes tracked in returned map

    if(!reserved_succeed){ // if failed to reserve cell found by pathfinding
                           // must update map with returned data so next closest cell can be reserved
        updateLocalMap(map_info, edge_info, map_status);
    }
        
    sem_post(acknowledgement_sem); // signalling to controller that the response data has been taken


    return reserved_succeed; // return whether robot can proceed to target cell
}