#include "RobotMaster.h"

RobotMaster::RobotMaster(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize): max_num_of_robots(num_of_robots){
    num_of_added_robots = 0; // initializing id counter to zero
    Message_Handler = r; // gathering request handler to use for receiving robot -> master communications
    
    maze_xsize = xsize;
    maze_ysize = ysize;

    GlobalMap = new GridGraph(maze_xsize, maze_ysize); // allocating GlobalMap to maze size

    number_of_unexplored_cells = 0;
}

RobotMaster::~RobotMaster(){
    delete GlobalMap; // deallocating GlobalMap
}

void RobotMaster::runRobotMaster(){ // function to continously run RobotMaster until the maze has been mapped
    
    robotMasterSetUp(); // calling set up function before receiving requests

    bool maze_mapped = false;

    // RobotMaster continues to receive requests until the maze is fully mapped
    while(!maze_mapped){
        maze_mapped = receiveRequests(); 
    } 

    return;
}
void RobotMaster::robotMasterSetUp(){
    accepting_requests = true;

    return;
}

bool RobotMaster::receiveRequests(){ // function to handle incoming requests from robots
    
    Message* request = Message_Handler->getMessage(); // gathering request from msg_queue
                                                      // pointer is gathered so response can be gathered by robot threads
                                                      // returned pointer will be NULL is no messages to get

    if(request != NULL){ // if there is a a request to handle, process it

        // gathering request type for switch statement
        num_of_receieve_transactions++; // get next request id for request tracking purposes

        if(accepting_requests){
            // processing incoming request
            handleIncomingRequest(request);
            
        }
        else if(!accepting_requests && request->msg_data->request_type == shutDownRequest_ID){ // shutDown confirmation request ( robot is telling master that it is shutting down)

            shutDownRequest(request);

            if(tracked_robots.size() == 0){ // if all robots have successfully shut down
                return true; // maze exploration done
            }
        }
        else{
            // not accepting requests but not a shut down request
            // thus do nothing
        }

        // request processed thus can delete data
        delete request->msg_data; // deleting dynamically allocated message data
        delete request; // delete dynamically allocated request
    }
    else{ // if no message to handle, do nothing

    }
    
    return false; // return false as maze is not completely mapped
}

RequestHandler* RobotMaster::getTargetRequestHandler(unsigned int target_id){ // gets a request handler for a specific robot
    for(int i = 0; i < tracked_robots.size(); i++){
        if(tracked_robots[i].robot_id == target_id){
            return tracked_robots[i].Robot_Message_Reciever; // return requesthandler for targer robot
        }
    }
    
    return NULL; // if not found, return NULL
}

void RobotMaster::shutDownRequest(Message* request){ // disconnects robot from system
    // shutDown request
    // [0] = type (unsigned int*), content: id of robot shutting down
    
    // no return data

    // gathering data from request
    m_shutDownRequest* request_data = (m_shutDownRequest*)request->msg_data; // first pointer of msg_data points to robot id
    

    // gathering response data
    m_shutDownResponse* response_data = new m_shutDownResponse;

    // sending response message to robot
    Message* response = new Message(t_Response, request->response_id); // creating new response with given response id      
    response->msg_data = response_data; // assigning response to message

    RequestHandler* robot_request_handler = getTargetRequestHandler(request_data->robot_id); // getting request handler to send response

    if (robot_request_handler != NULL){ // if request handler gathered send data
        robot_request_handler->sendMessage(response);

        // shut down request has been fully handled, can remove robot from system
        removeRobot(request_data->robot_id); 
    }
    else{ // if no request handler found, must delete response data and not send message
        delete response_data;
        delete response;
    }

    exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions);

    return;
}

void RobotMaster::addRobotRequest(Message* request){ // adds robot to controller system with any required information
    // addRobot request msg_data layout:
    // [0] = type: (unsigned int*), content: x coordinate of robot
    // [1] = type: (unsigned int*), content: y coordinate of robot
    // [2] = type: (RequestHandler*), content: message handler for master -> robot messages
    
    // return data
    // [0] = type: (unsigned int*), content: id to be assigned to robot

    // gathering incoming request
    m_addRobotRequest* request_data = (m_addRobotRequest*)request->msg_data;
    
    // processing request data
    unsigned int x = request_data->x;
    unsigned int y = request_data->y;
    RequestHandler* robot_request_handler = request_data->robot_request_handler;

    unsigned int robot_id = addRobot(x, y, robot_request_handler); // add robot using coordinates
                                                                  // return value is assigned id of robot

    // gathering response data
    m_addRobotResponse* response_data = new m_addRobotResponse;
    response_data->robot_id = robot_id;

    exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions); // adding request info to tracking JSON
    
    // sending response message to robot
    Message* response = new Message(t_Response, request->response_id); // creating new response with given response id
        
    response->msg_data = response_data; // assigning response to message

    robot_request_handler->sendMessage(response);

    return;
}

void RobotMaster::updateGlobalMapRequest(Message* request){
    // updateGlobalMap request msg_data layout:
    // [0] = type: (unsigned int*), content: id of robot sending request
    // [1] = type: (vector<bool>*), content: vector containing information on walls surrounding robot
            // [0] = north, [1] = south, [2] = east, [3] = west
            // 0 = connection, 1 = wall
    // [2] = type: (Coordinates*), content: current coordinates of where the read occured
    
    // return data = none

    // gathering incoming request data
    m_updateGlobalMapRequest* request_data = (m_updateGlobalMapRequest*)request->msg_data;

    unsigned int robot_id = request_data->robot_id;
    std::vector<bool> wall_info = request_data->wall_info;
    Coordinates cords = request_data->cords; // gathering position of scanned reading

    updateGlobalMap(&robot_id, &wall_info, &cords); // updating global map with information

    //request->return_data.push_back((void*)&robot_id); // telling Robot that data was successfully added to GlobalMap and it can continue

    // gathering response data
    m_updateGlobalMapResponse* response_data = new m_updateGlobalMapResponse;

    exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions); // adding request info to tracking JSON

    // sending response message to robot
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

void RobotMaster::updateRobotLocationRequest(Message* request){
    // updateRobotLocation request msg_data layout:
    // [0] = type: (unsigned int*), content: id of robot sending request
    // [1] = type: (Coordinates*), content: cell which robot now occupies

    // no return data

    // gathering incoming request data
    m_updateRobotLocationRequest* request_data = (m_updateRobotLocationRequest*)request->msg_data; 
    unsigned int robot_id = request_data->robot_id;
    Coordinates new_robot_location = request_data->new_robot_location;                    

    for(int i = 0; i < tracked_robots.size(); i++){
        if(tracked_robots[i].robot_id == robot_id){
            tracked_robots[i].robot_position = new_robot_location;
            break;
        }
    }

    // return data allocation
    m_updateRobotLocationResponse* response_data = new m_updateRobotLocationResponse; // response message

    exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions); // adding request info to tracking JSON

    // sending response message to robot
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

unsigned int RobotMaster::addRobot(unsigned int x, unsigned int y, RequestHandler* r){ // adding robot to control system
                                                                                       // this must be completed by all robots before beginning exploration
    
    number_of_unexplored_cells++; // incrementing number of unexplored by 1 as current robot cells has presumably not been explored

    num_of_added_robots++; // incrementing inorder to determine next id to give a robot

    RobotInfo temp; // buffer to store robot info before pushing it to the tracked_robots vecto

    temp.robot_id = num_of_added_robots; // assigning id to new robot entry 
    temp.robot_position.x = x;  // assigning position to new robot entry 
    temp.robot_position.y = y;
    temp.robot_status = 0;      // updating current robot status to 0 to leave it on stand by
    temp.Robot_Message_Reciever = r; // assigning Request handler for Master -> robot communications

    GlobalMap->nodes[y][x] = 2; // setting current position of robot to 2 as it has been seen but not explored until robot sends first scan update

    tracked_robots.push_back(temp); // adding robot info to tracked_robots

    return temp.robot_id; // returning id to be assigned to the robot which triggered this function
}

void RobotMaster::removeRobot(unsigned int id){
    for(int i = 0; i < tracked_robots.size(); i++){ // search for robot in tracked_robots
        if(tracked_robots[i].robot_id == id){ // if robot has been identified
            tracked_robots.erase(tracked_robots.begin()+i); // delete it from tracked_robots
            break;
        }
    }
    
    return;
}

void RobotMaster::updateGlobalMap(unsigned int* id, std::vector<bool>* connections, Coordinates* C){

    updateRobotLocation(id, C);

    //GlobalMapInfo[C->y][C->x].reserved = 0; // unreserving cell as it has been scanned

    if (GlobalMap->nodes[C->y][C->x] != 1){ // checking if there is a need to update map (has the current node been explored?)
        
        number_of_unexplored_cells--; // subtracting number of unexplored cells as new cell has been explored

        // updating vertical edges in GlobalMap using robot reading
        GlobalMap->y_edges[C->y][C->x] = (*connections)[0]; // north
        GlobalMap->y_edges[C->y + 1][C->x] = (*connections)[1]; // south

        GlobalMap->x_edges[C->y][C->x] = (*connections)[2]; // east
        GlobalMap->x_edges[C->y][C->x + 1] = (*connections)[3]; // west

        GlobalMap->nodes[C->y][C->x] = 1; // updating state of node to be 1 as it has been explored

        // now we will update the neighbouring cells to see if they have previously been explored
        // if not, they will be marked with a '2' on the GlobalMap Nodes Array

        if(!GlobalMap->y_edges[C->y][C->x] && GlobalMap->nodes[C->y - 1][C->x] == 0){ // checking if node to north hasn't been explored by a Robot
            GlobalMap->nodes[C->y - 1][C->x] = 2; // if unexplored and no wall between robot and cell, set northern node to unexplored
            number_of_unexplored_cells++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking south
        if(!GlobalMap->y_edges[C->y + 1][C->x] && GlobalMap->nodes[C->y + 1][C->x] == 0){ // checking if node to north hasn't been explored by a Robot
            GlobalMap->nodes[C->y + 1][C->x] = 2; // if unexplored and no wall between robot and cell, set southern node to unexplored
            number_of_unexplored_cells++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking east
        if(!GlobalMap->x_edges[C->y][C->x] && GlobalMap->nodes[C->y][C->x - 1] == 0){ // checking if node to north hasn't been explored by a Robot
            GlobalMap->nodes[C->y][C->x - 1] = 2; // if unexplored and no wall between robot and cell, set eastern node to unexplored
            number_of_unexplored_cells++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking west
        if(!GlobalMap->x_edges[C->y][C->x + 1] && GlobalMap->nodes[C->y][C->x + 1] == 0){ // checking if node to north hasn't been explored by a Robot
            GlobalMap->nodes[C->y][C->x + 1] = 2; // if unexplored and no wall between robot and cell, set western node to unexplored
            number_of_unexplored_cells++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
    }
    else{ // if there is no need to update map

    }

    printGlobalMap(); // print updated GlobalMap

    return;
}

std::vector<bool> RobotMaster::getNodeEdgeInfo(Coordinates* C){
    
    std::vector<bool> edge_info; // vector to return with information on edges surrounding node C

    //if(GlobalMap->nodes[C->y][C->x] == 1){ // if node is valid and seen, return connection information

        edge_info.push_back(GlobalMap->y_edges[C->y][C->x]);    // [0] = north edge
        edge_info.push_back(GlobalMap->y_edges[C->y + 1][C->x]);// [1] = south edge
        edge_info.push_back(GlobalMap->x_edges[C->y][C->x]);    // [2] = east edge
        edge_info.push_back(GlobalMap->x_edges[C->y][C->x + 1]);// [3] = west edge

        return edge_info;
   // }
    
    return edge_info; // return empty vector if node has not been explored yet

}

std::vector<Coordinates> RobotMaster::getSeenNeighbours(unsigned int x, unsigned  int y){ // function to gather seen neighbouring cells of a selected cell based on global map

    std::vector<Coordinates> ret_value; // vector of Coordinates to return
                                        // this will contain the coordinates of valid neighbouring nodes
    
    Coordinates buffer; // buffer structor to gather positions of neighbouring nodes before pushing to vector

    // check if neighbour to the north is valid and connected via an edge (no wall)
    if(y != 0){ // protection to ensure invalid part of nodes is not accessed if y = 0
        if(!GlobalMap->y_edges[y][x] && GlobalMap->nodes[y-1][x] > 0){
            buffer.x = x; 
            buffer.y = y - 1;
            ret_value.push_back(buffer);
        }
    }
    // check if neighbour to the south is valid and connected via an edge (no wall)
    if(y != maze_ysize - 1){
        if(!GlobalMap->y_edges[y+1][x] && GlobalMap->nodes[y+1][x] > 0){
            buffer.x = x; 
            buffer.y = y + 1;
            ret_value.push_back(buffer);
        }
    }
    // check if neighbour to the east is valid and connected via an edge (no wall)
    if(x != 0){ // protection to ensure invalid part of nodes is not accessed if x = 0
        if(!GlobalMap->x_edges[y][x] && GlobalMap->nodes[y][x-1] > 0){
            buffer.x = x - 1; 
            buffer.y = y;
            ret_value.push_back(buffer);
        }
    }
    // check if neighbour to the west is valid and connected via an edge (no wall)
    if(x != maze_xsize - 1){
        if(!GlobalMap->x_edges[y][x+1] && GlobalMap->nodes[y][x+1] > 0){
            buffer.x = x + 1; 
            buffer.y = y;
            ret_value.push_back(buffer);
        }
    }
    return ret_value; // returning vector
}

void RobotMaster::gatherPortionofMap(Coordinates curr_node, Coordinates neighbour_node, std::vector<Coordinates>* map_nodes, std::vector<std::vector<bool>>* map_connections, std::vector<char>* node_status){ // generates a portion of the map for transfer to robot using breadth first search

    std::queue<Coordinates> node_queue; // creating node queue to store nodes to be "explored" by algorithm

    // initializing value of queue and 
    node_queue.push(curr_node); // adding first node to explore to node queue
    
    // gathering starting cell info for return
    map_nodes->push_back(curr_node); // node coordinates
    map_connections->push_back(getNodeEdgeInfo(&curr_node)); // node connections
    node_status->push_back(GlobalMap->nodes[curr_node.y][curr_node.x]); // node status

    while(node_queue.size() != 0){// while nodes to explore are in node_queue
        
        curr_node = node_queue.front(); // gathering node from front of queue

        //printf("curr node: %d,%d\n", curr_node.x, curr_node.y);

        node_queue.pop(); // removing node from front of the queue as new nodes must be added to queue

        std::vector<Coordinates> valid_neighbours = getSeenNeighbours(curr_node.x, curr_node.y); // gathering neighbours of current node

        for(int i = 0; i < valid_neighbours.size(); i++){ // iterate through all of the current node's neighbours to see if they have been explored
            
            bool already_visited = false;

            for(int j = 0; j < map_nodes->size(); j++){
                if(valid_neighbours[i] == (*map_nodes)[j]){
                    already_visited = true;
                    break;
                }
            }

            if(!already_visited){
                map_nodes->push_back(valid_neighbours[i]); // node coordinates
                map_connections->push_back(getNodeEdgeInfo(&valid_neighbours[i])); // node connections
                node_status->push_back(GlobalMap->nodes[valid_neighbours[i].y][valid_neighbours[i].x]); // node status
            }
        }
    }

    return; // can return with map information
}

bool RobotMaster::checkIfOccupied(unsigned int x, unsigned int y, unsigned int* ret_variable){ // checks if a robot is within the cell passed into the function
                                                                                               // returns true is a robot is detected
                                                                                               // if a robot is found, ret_variable is modified to contain the id of the found robot
    unsigned int temp = 0;
    for(int i = 0; i < tracked_robots.size(); i++){ // looping through robots position
        if(tracked_robots[i].robot_position.x == x && tracked_robots[i].robot_position.y == y){ // if robot is occupying the location passed in
            temp = tracked_robots[i].robot_id; // returning found robot id
            *ret_variable = temp;

            return true; // return true as robot is occupying the cell
        }
    }
    return false; // returning false as robot is not found within the occupied cell
}

void RobotMaster::updateRobotLocation(unsigned int* id, Coordinates* C){ // updates the location of a robot to the location specified

    for(int i = 0; i < tracked_robots.size(); i++){ // finding robot to update
        if (tracked_robots[i].robot_id == *id){ // if robot found using id
            tracked_robots[i].robot_position = *C; // update position in RobotInfo
            
            // TODO: update occupying robot information in CellInfo matrix
            
            break; 
        }
    }
    return;
}
    
void RobotMaster::updateAllRobotState(int status){

    for(int i = 0; i < tracked_robots.size(); i++){ // creating messages to update state of all robots

        Message* messages = new Message(t_Request, -1); // creating new messages for each robot

        // assigning data to message
        m_updateRobotStateRequest* message_data = new m_updateRobotStateRequest;
        message_data->target_state = status;

        messages->msg_data = message_data; // specifying state to update all robots to
        
        tracked_robots[i].Robot_Message_Reciever->sendMessage(messages); // sending message
        tracked_robots[i].robot_status = status; // updating local robot information to current status
    }

    return;    
}

/*
void RobotMaster::printRequestInfo(Message* request){
    // gathering request information
    unsigned int request_id = request_id_tracker;
    int request_type = request->msg_data;
    
    //printing general request information
    printf("~~~~~\n");
    printf("Request %d\n", request_id);
    printf("Type = %d", request_type);
    
    switch(request_type){
        case -1: // shutDown confirmation request ( telling master robot has finished exploring)
        {
            printf(" - Shut Down Request\n");
            printf("~~~~~\n");
            printf("Robot Shutting Down: %d", request->msg_data[0]);
            
            break;
        }
        case 0: // addRobot request
        {
            printf(" - Add Robot Request\n");

            printf("~~~~~\n");
            printf("Request Data:\n");
            printf("Robot X Position: %d\n", request->msg_data[0]);
            printf("Robot Y Position: %d\n", request->msg_data[1]);

            printf("~~~~~\n");
            printf("Response Data:\n");
            printf("Robot Assigned id: %d", request->return_data[0]);
            
            break;
        }
        case 1: // updateGlobalMap request
        {   
            printf(" - Update Global Map Request\n");

            printf("~~~~~\n");
            printf("Request Data:\n");
            printf("Robot ID: %d\n", request->msg_data[0]);

            // TODO: print wall information

            printf("Scan Position: x - %d, y = %d\n");

            break;
        }
        case 2: // move2cell request
        {
            break;
        }
        case 3: // reserveCell request (robot wants to start exploring from a cell without other robots using it)
        {
            break;
        }
        case 4: // update Robot Location  (tells master that robot has completed move operation)
        {
            break;
        }
    }

    printf("~~~~~\n");
}*/

void RobotMaster::exportRequestInfo2JSON(m_genericRequest* request, m_genericRequest* response, unsigned int transaction_id){
    // creating jsons
    json request_buffer_json, response_buffer_json; // buffer jsons to store request and response information 

    switch(request->request_type){ // switch to determine which type of request has been received
                                   // this is required inorder to typecast and export the appropriate information
        case shutDownRequest_ID:
        {
            // typecast to appropriate child class to gather request data
            m_shutDownRequest* request_cast = (m_shutDownRequest*) request;
            
            // adding request infomation to buffer json
            request_buffer_json["ID"] = request_cast->robot_id;

            break;
        }
        case addRobotRequest_ID:
        {
            // typecast to appropriate child class to gather request data
            m_addRobotRequest* request_cast = (m_addRobotRequest*) request;
            
            // adding request infomation to buffer json
            request_buffer_json["x_pos"] = request_cast->x;
            request_buffer_json["y_pos"] = request_cast->y;

            break;
        }
        case updateGlobalMapRequest_ID:
        {
            // typecast to appropriate child class to gather request data
            m_updateGlobalMapRequest* request_cast = (m_updateGlobalMapRequest*) request;
            
            // adding request infomation to buffer json
            request_buffer_json["ID"] = request_cast->robot_id;

            request_buffer_json["Current_Cell"] = { {"x_pos", request_cast->cords.x},
                                                    {"y_pos", request_cast->cords.y} };

            // json.hpp does not allow for bool to be placed into json therefore must convert to other datatype
            // initialize all walls to no as they have not be checked
            std::string wall_north = "n"; 
            std::string wall_south = "n"; 
            std::string wall_east = "n";
            std::string wall_west = "n";

            if(request_cast->wall_info[0]) // if there is a wall to the north
                wall_north = "y"; // set wall_north to y for yes
            
            if(request_cast->wall_info[1]) // if there is a wall to the south
                wall_south = "y"; // set wall_north to y for yes
            
            if(request_cast->wall_info[2]) // if there is a wall to the east
                wall_east = "y"; // set wall_north to y for yes

            if(request_cast->wall_info[3]) // if there is a wall to the west
                wall_west = "y"; // set wall_north to y for yes

            // adding wall info to buffer json
            request_buffer_json["Wall_Info"] = { {"North", wall_north},
                                                 {"South", wall_south},
                                                 {"East", wall_east},
                                                 {"West", wall_west} };
            break;
        }
        case move2CellRequest_ID:
        {
            // typecast to appropriate child class to gather request data
            m_move2CellRequest* request_cast = (m_move2CellRequest*) request;
            
            // adding request infomation to buffer json
            request_buffer_json["ID"] = request_cast->robot_id;

            request_buffer_json["Target_Cell"] = { { "x_pos", request_cast->target_cell.x},
                                                   { "y_pos", request_cast->target_cell.y} };
            break;
        }
        case reserveCellRequest_ID:
        {
             // typecast to appropriate child class to gather request data
            m_reserveCellRequest* request_cast = (m_reserveCellRequest*) request;
            
            // adding request infomation to buffer json
            request_buffer_json["ID"] = request_cast->robot_id;

            request_buffer_json["Target_Cell"] = { { "x_pos", request_cast->target_cell.x},
                                                   { "y_pos", request_cast->target_cell.y} };            

            request_buffer_json["Neighbouring_Cell"] = { { "x_pos", request_cast->neighbouring_cell.x},
                                                         { "y_pos", request_cast->neighbouring_cell.y} };   
            break;
        }
        case updateRobotLocationRequest_ID:
        {
             // typecast to appropriate child class to gather request data
            m_updateRobotLocationRequest* request_cast = (m_updateRobotLocationRequest*) request;
            
            // adding request infomation to buffer json
            request_buffer_json["ID"] = request_cast->robot_id;

            request_buffer_json["Current_Cell"] = { { "x_pos", request_cast->new_robot_location.x},
                                                    { "y_pos", request_cast->new_robot_location.y} };
            break;
        }
    }

    switch(response->request_type){ // switch to determine which type of response is being sent
                                   // this is required inorder to typecast and export the appropriate information
        case shutDownRequest_ID:
        {
            // no response required thus no print out

            break;
        }
        case addRobotRequest_ID:
        {
            // typecast to appropriate child class to gather response data
            m_addRobotResponse* response_cast = (m_addRobotResponse*) response;
            
            // adding request infomation to buffer json
            response_buffer_json["Assigned_ID"] = response_cast->robot_id;

            break;
        }
        case updateGlobalMapRequest_ID:
        {
            // no response required thus no print out

            break;
        }
        case move2CellRequest_ID:
        {
            // typecast to appropriate child class to gather response data
            m_move2CellResponse* response_cast = (m_move2CellResponse*) response;
            
            // need to convert bool info into y/n
            char can_movement_occur = 'n';
            if (response_cast->can_movement_occur)
                can_movement_occur = 'y';

            // adding request infomation to buffer json
            response_buffer_json["Movement_Occured"] = can_movement_occur;
            
            break;
        }
        case reserveCellRequest_ID:
        {
            // typecast to appropriate child class to gather response data
            m_reserveCellResponse* response_cast = (m_reserveCellResponse*) response;
            
            // need to convert bool info into y/n
            std::string cell_reserved = "n";
            if (response_cast->cell_reserved)
                cell_reserved = "y";

            // adding request infomation to buffer json
            response_buffer_json["Cell_Reserved"] = cell_reserved;

            break;
        }
        case updateRobotLocationRequest_ID:
        {
            // no response required thus no print out

            break;
        }
    }

    // transaction information has been encapsulated into two seperate jsons
    // they can now be combined in a buffer json and then added to the main "RequestInfo" JSON
    json request_json;
    request_json["Type"] = request->request_type;
    request_json["Request"] = request_buffer_json;
    request_json["Response"] = response_buffer_json;
    request_json["Transaction"] = transaction_id;
    
    RequestInfo.push_back(request_json); // adding generated json to RequestInfo json

    return; 
}

bool RobotMaster::printGlobalMap(){ // function to print global map of maze including robot location
                                     // maze design based off what can be seen here: https://www.chegg.com/homework-help/questions-and-answers/using-c-1-write-maze-solving-program-following-functionality-note-implementation-details-a-q31826669

    std::string logos[9] = { "   ", "---", "|", " ", " R ", " . ", " X ", " * "}; // array with logos to use when printing maze
    
    if(maze_xsize == 0 || maze_ysize == 0){ // if maze has not been allocated
        printf("Error: Maze size has not been specified\n");
        return false; // return false as printing failed
    }

    printf("*Global Map*\n"); // printing title and maze information

    int string_pointer = 0; // integer used to determine which logo needs to be printed from logo vector

    int count = 0; // counter to determine if both the column and row edges have been printed
    int i = 0; // counter to track if the whole maze has been printed

    while(!(i >= maze_ysize && count == 1)){ // while loop to determing which row to print (i = node row number)
    
        if(count == 0){ // printing the horizontal walls of maze

            for(int j = 0; j < maze_xsize; j++){

                if(GlobalMap->y_edges[i][j]){ // if there is no edge between two nodes
                    string_pointer = 1; // print horizontal line
                }
                else{ // if there is an edge between two nodes
                    string_pointer = 0; // print horizontal line
                }

                printf("+%s", logos[string_pointer].c_str());
            }
            printf("+");
        }
        else{ // printing vertical walls of the maze

            for(int j = 0; j < maze_xsize + 1; j++){

                // checking the walls between two nodes (e.g. wall?, no wall?)
                if(GlobalMap->x_edges[i][j]){ // if there is no edge between two nodes
                    string_pointer = 2; // print horizontal line
                }
                else{ // if there is an edge between two nodes
                    string_pointer = 3; // print horizontal line
                }
                printf("%s", logos[string_pointer].c_str());

                unsigned int* found_id = new unsigned int; // variable to store id of robot if found using checkIfOccupied in the second else if statement

                // checking contents of a node (e.g. does it have a robot?)
                if (j >= maze_xsize){ // if iterating outside of valid node
                    string_pointer = 3; // print empty space as node is outside of maze
                }
                else if(checkIfOccupied(j, i, found_id)){ // if current node is the robot's location
                    printf("%2d ", *found_id); // printing id number of robot within the cell
                                               // TODO: print 3 width digit numbers in centre of cell without error
                    string_pointer = -1; // print nothing after this ifelse statement as the printing has been handled locally
                }
                else if(GlobalMap->nodes[i][j] == 0){ // if current node is invalid (unseen and unexplored)
                    string_pointer = 6; // print I for invalid cell
                }
                else if(GlobalMap->nodes[i][j] == 2){ // if current node has been seen but not explored
                    string_pointer = 7; // print * for seen node
                }
                else{ // if current cell has been seen and explored (valid)
                    string_pointer = 0; // print empty space
                }
                if (string_pointer != -1) // if printing is needed
                    printf("%s",logos[string_pointer].c_str());

                delete found_id; // deleting found_id as dynamically allocated
            }
        }
        
        if (count == 1){ // if both the row and column corresponding to i value have been printed
            count = 0; // reset count value
            i++; // increment i to access next row
        }
        else // if only the column edges corresponding to i have been printed
            count++;

        printf("\n");

    }

    return true; // return true as printing succeeded
}