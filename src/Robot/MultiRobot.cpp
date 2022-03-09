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

void MultiRobot::assignIdFromMaster(){

    printf("ROBOT: calling robot master\n");
    // sending addRobot request to RobotMaster to get ID
    Message* temp_message = new Message(); // buffer to load data into before sending message

    temp_message->request_type = 0; // request_type = 0 as addRobot request required

    temp_message->msg_data.push_back((void*) &x_position); // adding x position of robot to [0]
    temp_message->msg_data.push_back((void*) &y_position); // adding y position of robot to [1]
    temp_message->msg_data.push_back((void*) Master_2_Robot_Message_Handler); // adding request handler for master -> robot message to [2]

    temp_message->res_sem = response_sem; // attaching robot communication semaphores to message
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    printf("ROBOT: waiting for id\n");

    sem_wait(response_sem); // waiting for response to be ready from controller

    id = *(unsigned int*)temp_message->return_data[0]; // gather assigned id from RobotMaster's response
    
    sem_post(acknowledgement_sem); // signalling to controller that the response has been utilised 

    printf("ROBOT: id = %d %x\n",id, temp_message);

    return;
}


int MultiRobot::getRequestsFromMaster(int status){ // checking if RobotMaster wants robot to change states
    
    Message* request = Master_2_Robot_Message_Handler->getMessage(); // gathering request from msg_queue
                                                                     // pointer is gathered so response can be gathered by robot threads
                                                                     // returned pointer will be NULL is no messages to get
    
    int ret_variable; // return variable to update status in RobotLoop

    if(request != NULL){ // if there is a a request to handle, process it
        switch (request->request_type){ // determining type of request before processing

                case -1: // change state to shut down
                    {  
                        // if a message of this type is recieved, no message contents
                        // only the response semaphore is used to tell the robot to shut down

                        ret_variable = request->request_type; // gathering status from request type
                             
                        break;
                    }
                
                case 0: // change state to standby mode
                    {
                        ret_variable = request->request_type;
                        
                        break;
                    }
                case 1: // change state to begin exploring
                    {
                        printf("ROBOT %d: Begin Exploring\n", id);
                        // if a message of this type is recieved, no message contents
                        // only the response semaphore is used to signal that the robot will begin exploring
                        ret_variable = request->request_type; // gathering status from request type
                        
                        break;
                    }
                default: // TODO: Implement handling if improper messge type is recieved
                    {
                        break;
                    }
            }

            delete request; // deleting recieved message as no longer needed
    }
    else{ // if no message to handle, do nothing
        ret_variable = status; // set status as current status
    }
    
    return ret_variable;
}

bool MultiRobot::requestMove2Cell(Coordinates target_cell){
    // need to message controller to see if target cell is occupied
    // if unoccupied, robot can move to cell
    // if cell is unoccupied (e.g. robot can move to cell), return true
    // if cell is occupied, return false
    Message* temp_message = new Message;

    temp_message->request_type = 2; // request_type = 2 as moveRequest required

    temp_message->msg_data.push_back((void*) &id); // adding id of robot to [0]
    temp_message->msg_data.push_back((void*) &target_cell); // adding target destination of robot to [1]

    temp_message->res_sem = response_sem; // attaching robot communication semaphores to message
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    sem_wait(response_sem); // waiting for response to be ready from controller

    bool is_not_occupied = (bool)temp_message->return_data[0]; // gather assigned id from RobotMaster's response
    
    sem_post(acknowledgement_sem); // signalling to controller that the response has been utilised 
    
    return is_not_occupied;
}

void MultiRobot::requestShutDown(){
    // sending message to notidy RobotMaster that robot is ready to shutdown
    Message* temp_message = new Message(); // buffer to load data into before sending message

    temp_message->request_type = -1; // request_type = -1 as shutDown request

    temp_message->msg_data.push_back((void*) &id); // adding id of robot sending request to [0]

    temp_message->res_sem = response_sem; // attaching communication semaphores to message
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    sem_wait(response_sem); // waiting for Master to acknowledge shutdown

    return;
}

void MultiRobot::requestRobotLocationUpdate(){
    // sending message with new robot position
    Message* temp_message = new Message(); // buffer to load data into before sending message

    temp_message->request_type = 4; // request_type = 1 as updateGlobalMap request required

    Coordinates robot_cords(x_position,y_position);// gathering robots current coordinates

    temp_message->msg_data.push_back((void*) &id); // adding id of robot sending request to [0]
    temp_message->msg_data.push_back((void*) &robot_cords); // adding robot location to [1]

    temp_message->res_sem = response_sem; // attaching communication semaphores to message
    temp_message->ack_sem = acknowledgement_sem;
    
    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

    sem_wait(response_sem); // waiting for data to be inputted into Master before continuing
    sem_post(acknowledgement_sem);

    return;
}

void MultiRobot::requestGlobalMapUpdate(std::vector<bool> connection_data){
    // sending message with scanned maze information
    Message* temp_message = new Message(); // buffer to load data into before sending message

    temp_message->request_type = 1; // request_type = 1 as updateGlobalMap request required

    Coordinates robot_cords(x_position,y_position);// gathering robots current coordinates

    temp_message->msg_data.push_back((void*) &id); // adding id of robot sending request to [0]
    temp_message->msg_data.push_back((void*) &connection_data); // adding vector containing information on walls surrounding robot to [1]
    temp_message->msg_data.push_back((void*) &robot_cords); // current coordinates of where the read occured

    temp_message->res_sem = response_sem; // attaching communication semaphores to message
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

    temp_message->request_type = 3; // request_type = 2 as moveRequest required

    temp_message->msg_data.push_back((void*) &id); // adding id of robot to [0]
    temp_message->msg_data.push_back((void*) &(planned_path[planned_path.size() - 1])); // adding target destination of robot to [1]

    Coordinates c(x_position,y_position); // gathering current robot coordinates as it may be needed 
    if(planned_path.size() > 1){ // if more than one element in planned_path, must pass second last element in deque as neigbouring cell
        temp_message->msg_data.push_back((void*) &(planned_path[planned_path.size() - 2])); // passing second last element in planned_path as neighbouring cell
    }
    else{ // as planned_path has only 1 element, cell must be neighbouring current position thus pass current position as neighbouring cell
        temp_message->msg_data.push_back((void*) &c); // passing current robot position as neighbouring cell
    }

    temp_message->res_sem = response_sem; // attaching robot communication semaphores to message
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    sem_wait(response_sem); // waiting for response to be ready from controller

    bool reserved_succeed = (bool)temp_message->return_data[0]; // gather whether cell has been reserved
    std::vector<Coordinates>* map_info = (std::vector<Coordinates>*)  temp_message->return_data[1]; // gather map nodes for map update
    std::vector<std::vector<bool>>* edge_info = (std::vector<std::vector<bool>>*)  temp_message->return_data[2]; 
    std::vector<char>* map_status = (std::vector<char>*)  temp_message->return_data[3];

    if(!reserved_succeed){ // if failed to reserve cell found by pathfinding
                           // must update map with returned data so next closest cell can be reserved
        updateLocalMap(map_info, edge_info, map_status);
    }
        
    sem_post(acknowledgement_sem); // signalling to controller that the response data has been taken


    return reserved_succeed; // return whether robot can proceed to target cell
}

