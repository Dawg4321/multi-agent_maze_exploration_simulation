#include "RobotMaster.h"

RobotMaster::RobotMaster(RequestHandler* r, int num_of_robots, unsigned int xsize, unsigned int ysize): num_of_robots(num_of_robots){
    
    Message_Handler = r; // gathering request handler to use for receiving robot -> master communications
    
    maze_xsize = xsize;
    maze_ysize = ysize;

    GlobalMap = new GridGraph(maze_xsize, maze_ysize); // allocating GlobalMap to maze size
    
    for(int i = 0; i < GlobalMap->x_edges.size(); i++) // placing wall in every cell location as they are unknown
        for(int j = 0; j < GlobalMap->x_edges[i].size(); j++)
            GlobalMap->x_edges[i][j] = true;

    for(int i = 0; i < GlobalMap->y_edges.size(); i++) // placing wall in every cell location as they are unknown
        for(int j = 0; j < GlobalMap->y_edges[i].size(); j++)
            GlobalMap->y_edges[i][j] = true;

    num_of_receieve_transactions = 0; // no transactions recieved yet
    number_of_frontier_cells = 0; // no cells have been explored

    num_of_added_robots = 0; // no robots have been added
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
    Message* response = new Message(t_Response, request->transaction_id); // creating new response with given response id      
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

void RobotMaster::addRobotRequest(Message* request){ // adds robot to master system with any required information
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
    Message* response = new Message(t_Response, request->transaction_id); // creating new response with given response id
        
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

    exportRequestInfo2JSON(request_data, NULL, num_of_receieve_transactions); // adding request info to tracking JSON

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

    updateRobotLocation(&robot_id, &new_robot_location); // updating robot location

    exportRequestInfo2JSON(request_data, NULL, num_of_receieve_transactions); // adding request info to tracking JSON

    return;
}

unsigned int RobotMaster::addRobot(unsigned int x, unsigned int y, RequestHandler* r){ // adding robot to control system
                                                                                       // this must be completed by all robots before beginning exploration
    num_of_added_robots++; // incrementing inorder to determine next id to give a robot

    RobotInfo temp; // buffer to store robot info before pushing it to the tracked_robots vecto

    temp.robot_id = num_of_added_robots; // assigning id to new robot entry 
    temp.robot_position.x = x;  // assigning position to new robot entry 
    temp.robot_position.y = y;
    temp.starting_position = temp.robot_position; // assigning starting position to robot
    temp.robot_moving = false; // initializing robot_moving flag to false as robot has not begun moving
    temp.robot_target = NULL_COORDINATE;  // setting target to an invalid coordinate as robots have not begun exploring

    temp.Robot_Message_Reciever = r; // assigning Request handler for Master -> robot communications

    if(GlobalMap->nodes[y][x] != 2){ // if the cell has not been marked as seen (e.g. another robot hasnt already been placed in the cell)
        GlobalMap->nodes[y][x] = 2; // setting current position of robot to 2 as it has been seen but not explored until robot sends first scan update
        number_of_frontier_cells++; // incrementing number of unexplored by 1 as current robot cells has presumably not been explored
    }

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


    if (GlobalMap->nodes[C->y][C->x] != 1){ // checking if there is a need to update map (has the current node been explored?)
        
        number_of_frontier_cells--; // subtracting number of unexplored cells as new cell has been explored

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
            number_of_frontier_cells++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking south
        if(!GlobalMap->y_edges[C->y + 1][C->x] && GlobalMap->nodes[C->y + 1][C->x] == 0){ // checking if node to north hasn't been explored by a Robot
            GlobalMap->nodes[C->y + 1][C->x] = 2; // if unexplored and no wall between robot and cell, set southern node to unexplored
            number_of_frontier_cells++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking east
        if(!GlobalMap->x_edges[C->y][C->x] && GlobalMap->nodes[C->y][C->x - 1] == 0){ // checking if node to north hasn't been explored by a Robot
            GlobalMap->nodes[C->y][C->x - 1] = 2; // if unexplored and no wall between robot and cell, set eastern node to unexplored
            number_of_frontier_cells++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking west
        if(!GlobalMap->x_edges[C->y][C->x + 1] && GlobalMap->nodes[C->y][C->x + 1] == 0){ // checking if node to north hasn't been explored by a Robot
            GlobalMap->nodes[C->y][C->x + 1] = 2; // if unexplored and no wall between robot and cell, set western node to unexplored
            number_of_frontier_cells++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
    }
    else{ // if there is no need to update map

    }

    // printGlobalMap(); // print updated GlobalMap

    return;
}

std::vector<bool> RobotMaster::getNodeEdgeInfo(Coordinates* C){
    
    std::vector<bool> edge_info; // vector to return with information on edges surrounding node C

    edge_info.push_back(GlobalMap->y_edges[C->y][C->x]);    // [0] = north edge
    edge_info.push_back(GlobalMap->y_edges[C->y + 1][C->x]);// [1] = south edge
    edge_info.push_back(GlobalMap->x_edges[C->y][C->x]);    // [2] = east edge
    edge_info.push_back(GlobalMap->x_edges[C->y][C->x + 1]);// [3] = west edge

    return edge_info;
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

    node_queue.push(curr_node); // adding first node to explore to node queue
    
    // gathering starting cell info for return
    map_nodes->push_back(curr_node); // node coordinates
    map_connections->push_back(getNodeEdgeInfo(&curr_node)); // node connections
    node_status->push_back(GlobalMap->nodes[curr_node.y][curr_node.x]); // node status

    while(node_queue.size() != 0){// while nodes to explore are in node_queue
        
        curr_node = node_queue.front(); // gathering node from front of queue

        node_queue.pop(); // removing node from front of the queue as new nodes must be added to queue

        std::vector<Coordinates> valid_neighbours = getSeenNeighbours(curr_node.x, curr_node.y); // gathering neighbours of current node

        for(int i = 0; i < valid_neighbours.size(); i++){ // iterate through all of the current node's neighbours to see if they have been explored
            
            bool already_visited = false;

            for(int j = 0; j < map_nodes->size(); j++){ // checking to see if the current node has already been visited
                if(valid_neighbours[i] == (*map_nodes)[j]){
                    already_visited = true;
                    break;
                }
            }

            if(!already_visited && valid_neighbours[i] != neighbour_node){ // if it hasnt been visited and is not down the path the robot came from, add it to map connections to send back to robot
                node_queue.push(valid_neighbours[i]); // add node to explore down during next iteration of the loop

                map_nodes->push_back(valid_neighbours[i]); // node coordinates
                map_connections->push_back(getNodeEdgeInfo(&valid_neighbours[i])); // node connections
                node_status->push_back(GlobalMap->nodes[valid_neighbours[i].y][valid_neighbours[i].x]); // node status
            }
        }
    }

    return; // can return as map information loaded into various vectors
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
            tracked_robots[i].robot_moving = false; // setting robot_moving flag to false as robot is done moving
            if(tracked_robots[i].planned_path.size() > 0)
                tracked_robots[i].planned_path.pop_front(); // remove front of planned_path as movement has occured
            
            break; 
        }
    }

    return;
}

void RobotMaster::updateAllRobotState(int status){

    for(int i = 0; i < tracked_robots.size(); i++){ // creating messages to update state of all robots
        
        updateRobotState(status, tracked_robots[i].Robot_Message_Reciever);
    }

    return;    
}

void RobotMaster::updateRobotState(int status, RequestHandler* Target_Robot_Receiver){

    Message* messages = new Message(t_Request, -1); // creating new messages for each robot

    // assigning data to message
    m_updateRobotStateRequest* message_data = new m_updateRobotStateRequest;
    message_data->target_state = status;

    messages->msg_data = message_data; // specifying state to update all robots to
    
    Target_Robot_Receiver->sendMessage(messages); // sending message

    return;
}

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

            for(int i = 0; i < request_cast->planned_path.size(); i++){
                request_buffer_json["Target_Cell"].push_back({ request_cast->planned_path[i].x, request_cast->planned_path[i].y });
            }
            
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
        case getMapRequest_ID:
        {
            // typecast to appropriate child class to gather response data
            m_getMapRequest* request_cast = (m_getMapRequest*) request;

            // adding request infomation to buffer json
            request_buffer_json["ID"] = request_cast->robot_id;

            request_buffer_json["Current_Cell"] = { { "x_pos", request_cast->current_cell.x},
                                                    { "y_pos", request_cast->current_cell.y} }; 
            
            request_buffer_json["Target_Cell"] = { { "x_pos", request_cast->target_cell.x},
                                                   { "y_pos", request_cast->target_cell.y} };
            break;
        }
    }
    if(response != NULL){
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
            case getMapRequest_ID:
            {
                // no data loaded into response as only map information is in response
                break;
            }
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

std::string RobotMaster::printGlobalMap(){ // function to print global map of maze including robot location
                                     // maze design based off what can be seen here: https://www.chegg.com/homework-help/questions-and-answers/using-c-1-write-maze-solving-program-following-functionality-note-implementation-details-a-q31826669
    
    if(maze_xsize == 0 || maze_ysize == 0){ // if maze has not been allocated
        throw "Critical Error: Cannot print maze as it has not been allocated";
    }

    std::string logos[9] = { "   ", "---", "|", " ", " R ", " . ", " X ", " * "}; // array with logos to use when printing maze

    int string_pointer = 0; // integer used to determine which logo needs to be printed from logo vector

    int count = 0; // counter to determine if both the column and row edges have been printed
    int i = 0; // counter to track if the whole maze has been printed

    std::stringstream string_stream; // string stream object to store maze before printout

    while(!(i >= maze_ysize && count == 1)){ // while loop to determing which row to print (i = node row number)
    
        if(count == 0){ // printing the horizontal walls of maze

            for(int j = 0; j < maze_xsize; j++){
                if(GlobalMap->y_edges[i][j]){ // if there is no edge between two nodes
                    string_pointer = 1; // print horizontal line
                }
                else{ // if there is an edge between two nodes
                    string_pointer = 0; // print horizontal line
                }
                string_stream << fmt::format("+{}", logos[string_pointer]);;
            }
            string_stream << "+";
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
                string_stream << fmt::format("{}", logos[string_pointer]);

                unsigned int* found_id = new unsigned int; // variable to store id of robot if found using checkIfOccupied in the second else if statement

                // checking contents of a node (e.g. does it have a robot?)
                if (j >= maze_xsize){ // if iterating outside of valid node
                    string_pointer = 3; // print empty space as node is outside of maze
                }
                else if(checkIfOccupied(j, i, found_id)){ // if current node is the robot's location
                    string_stream << fmt::format("{0:2d} ", *found_id);
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
                if (string_pointer != -1){ // if printing is needed
                    string_stream << fmt::format("{}",logos[string_pointer]);
                }

                delete found_id; // deleting found_id as dynamically allocated
            }
        }
        
        if (count == 1){ // if both the row and column corresponding to i value have been printed
            count = 0; // reset count value
            i++; // increment i to access next row
        }
        else{ // if only the column edges corresponding to i have been printed
            count++;
        }

        string_stream << "\n";
    }

    std::cout << string_stream.str(); // printing string from string stream
    
    return string_stream.str(); // returning string of printout for later usage
}

void RobotMaster::setGlobalMap(GridGraph* g){ 
    *GlobalMap = *g;
    
    for(int i = 0; i < GlobalMap->nodes.size(); i++){ // need to account for all unexplored cells in new map
        for(int j = 0; j < GlobalMap->nodes[i].size(); j++){
            if(GlobalMap->nodes[i][j] == 2)
                number_of_frontier_cells++;
        }
    }
}

RobotInfo* RobotMaster::getRobotInfo(unsigned int id){

    for(int i = 0; i < tracked_robots.size(); i++){
        if(tracked_robots[i].robot_id == id){ // if robot found
            return &tracked_robots[i]; // return pointer to RobotInfo
        }
    }

    return NULL; // if robot not found, return NULL pointer
}