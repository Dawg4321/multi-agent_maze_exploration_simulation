#include "RobotMaster_C.h"

RobotMaster_C::RobotMaster_C(){

}

RobotMaster_C::~RobotMaster_C(){

}

void RobotMaster_C::move2CellRequest(Message* request){

    // gathering incoming request
    m_move2CellRequest* request_data = (m_move2CellRequest*)request->msg_data; 
    
    unsigned int robot_id = request_data->robot_id;
    Coordinates target_cell = request_data->target_cell;

    m_move2CellResponse* response_data = new m_move2CellResponse;

    RobotInfo* current_robot_info = getRobotInfo(robot_id);

    if(current_robot_info->planned_path.size() == 0){ // if there is no planned path and the robot is attempting to move, movement request is stale as another path needs to be planned thus no need to check for collision
        return;
    }

    for(int i = 0; i < tracked_robots.size(); i++){ // simple test to ensure to robots are occupying the same location (this is a critical error)
        if(tracked_robots[i].robot_position == current_robot_info->robot_position && current_robot_info->robot_id != tracked_robots[i].robot_id && current_robot_info->robot_position != Coordinates(0,0))
            break;
    }

    RobotInfo* robot_causing_collision = checkForCollision(&target_cell, robot_id); // check and find robot information from robot who is causing a collision  

    if(isRobotMoving(target_cell, current_robot_info->robot_id)){ // checking to see if another robot is currently in the process of moving into the target cell
        response_data->can_movement_occur = false;
    }
    else if(robot_causing_collision == NULL){ // if no robot was found to be causing a collision
        current_robot_info->robot_moving = true; // setting robot_moving flag to true as robot is now moving
        response_data->can_movement_occur = true; // update message to notify robot that movement can occur
    }
    else if(robot_causing_collision->robot_id == current_robot_info->robot_id){ // if the robot causing the collision is the current robot
        printf("Critical Error: Robot attempting to move to a cell it already occupies\n"); // critical error message
        response_data->can_movement_occur = false; //  update message to notify robot that movement can't occur
    }
    else{ // target cell is occupied by another robot, try to "job swap"
        if(robot_causing_collision->planned_path.size() == 0 && GlobalMap->nodes[robot_causing_collision->robot_position.y][robot_causing_collision->robot_position.x] == 1){ // if robot causing collision has no job (e.g. is stationary)

            exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions); // no response will be sent thus adding request info to tracking JSON

            delete response_data; // deleting response data as it is not needed (new requests will be sent instead of a response)

            setTargetCellRequest(current_robot_info->robot_target, robot_causing_collision->robot_id); // telling collision robot to plan a path to current robot's target
            
            current_robot_info->planned_path.clear(); // clearing current robot's planned path as it must find a path to its new target
            robot_causing_collision->planned_path.clear(); // clearing collision robot's planned path as it must find a path to its new target

            robot_causing_collision->robot_target = current_robot_info->robot_target; // giving robot_causing_collision current robot's target
            current_robot_info->robot_target = NULL_COORDINATE; // as robot causing collision has no target to swap, set robot's target as its current cell

            // telling current robot to find a new target as collision robot has not target cell
            updateRobotState(2, current_robot_info->Robot_Message_Reciever); // tell robot to attempt to reserve another cell

            return; // can return as no need to send response as setTargetRequest will invalidate it
        }
        else if(robot_causing_collision->robot_position == target_cell && robot_causing_collision->planned_path.size() > 0 && robot_causing_collision->planned_path[0] == current_robot_info->robot_position){
            
            exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions); // no response will be sent thus adding request info to tracking JSON

            delete response_data; // deleting response data as it is not needed (new requests will be sent instead of a response)
            
            // sending request for robots to swap jobs
            setTargetCellRequest(current_robot_info->robot_target, robot_causing_collision->robot_id); // telling collision robot to plan a path to current robot's target
            setTargetCellRequest(robot_causing_collision->robot_target, current_robot_info->robot_id); // telling current robot to plan a path to collision robot's target

            current_robot_info->planned_path.clear(); // clearing current robot's planned path as it must find a path to its new target
            robot_causing_collision->planned_path.clear(); // clearing collision robot's planned path as it must find a path to its new target

            Coordinates target_buffer = current_robot_info->robot_target; // swapping targets in tracked_robots
            current_robot_info->robot_target = robot_causing_collision->robot_target;
            robot_causing_collision->robot_target = target_buffer;
            

            return; // can return as no need to send response as setTargetRequest will invalidate it
        }
        else{ // if target cell is occupied by another robot and waiting for next robot's may solve the problem
            response_data->can_movement_occur = false; //  update message to notify robot  that movement can't occur
        }
    }

    exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions); // adding request info to tracking JSON

    // attaching response data to message
    Message* response = new Message(t_Response, request->response_id); // creating new response with passed in response id

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

void RobotMaster_C::getMapRequest(Message* request){
    
    // gathering incoming request
    m_getMapRequest* request_data = (m_getMapRequest*)request->msg_data;

    unsigned int robot_id = request_data->robot_id; // gathering robot id
    Coordinates target_cell = request_data->target_cell; // gather target cell which robot wants a map to
    Coordinates current_cell = request_data->current_cell; // gather current locaiton of roboy

    m_getMapResponse* response_data = new m_getMapResponse;

    gatherMap2Target(current_cell, target_cell, &response_data->map_coordinates, &response_data->map_connections, &response_data->map_status); // gathering portion of map and placing in various passed vectors

    exportRequestInfo2JSON(request_data, response_data, num_of_receieve_transactions); // adding request info to tracking JSON

    // attaching response data to message
    Message* response = new Message(t_Response, request->response_id); // creating new response with passed in response id

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

void RobotMaster_C::gatherMap2Target(Coordinates current_node, Coordinates target_node, std::vector<Coordinates>* map_nodes, std::vector<std::vector<bool>>* map_connections, std::vector<char>* node_status){ // gathers portion of the map for transfer to robot using breadth first search

    Coordinates curr_node = current_node; // gather current node of robot into curr_node for map BFS search 

    std::queue<Coordinates> node_queue; // creating node queue to store nodes to be "explored" by algorithm
    std::map<Coordinates, Coordinates> visited_nodes; // map to track explored nodes to their parent node

    node_queue.push(curr_node); // adding first node to explore to node queue
    visited_nodes.insert({curr_node, curr_node}); // adding first node to visited node maps using itself as parent
    
    // gathering starting cell info for return
    map_nodes->push_back(curr_node); // node coordinates
    map_connections->push_back(getNodeEdgeInfo(&curr_node)); // node connections
    node_status->push_back(GlobalMap->nodes[curr_node.y][curr_node.x]); // node status

    while(node_queue.size() != 0){// while nodes to explore are in node_queue
        
        curr_node = node_queue.front(); // gathering node from front of queue

        node_queue.pop(); // removing node from front of the queue as new nodes must be added to queue

        if(curr_node == target_node){
            break;
        }

        std::vector<Coordinates> valid_neighbours = RobotMaster::getSeenNeighbours(curr_node.x, curr_node.y); // gathering neighbours of current node

        for(int i = 0; i < valid_neighbours.size(); i++){ // iterate through all of the current node's neighbours to see if they have been explored
            

            bool node_in_map = false; // variable to track whether neighbour is in map
            
            for(auto [key, val]: visited_nodes){ // iterate through visited_nodes
                if (key == valid_neighbours[i]){ // if current neighbour is in map
                    node_in_map = true;          // set node_in_map
                    break;
                }
            }
            if(!node_in_map){ // if neighbour not found in map
                node_queue.push(valid_neighbours[i]); // add to node_queue and visited nodes
                visited_nodes.insert({valid_neighbours[i], curr_node}); 
            }
        }
    }

    // must out map information of cells into various return vectors
    
    curr_node = target_node; // ensuring the current node is pointing to destination
                             // this allows for the "walk back" through nodes to start to occur
    
    while(curr_node != current_node){ // while the starting node has not been found from walk back

        // loading next node into various return vectors
        map_nodes->push_back(curr_node); // node coordinates
        map_connections->push_back(getNodeEdgeInfo(&curr_node)); // node connections
        node_status->push_back(GlobalMap->nodes[curr_node.y][curr_node.x]); // node status

        for(auto [key, val]: visited_nodes){ // selecting parent node
            if (key == curr_node){
                curr_node = val;
                break;
            }
        }
    }

    return; // can return as map information loaded into various vectors
}

RobotInfo* RobotMaster_C::checkForCollision(Coordinates* movement_cell, unsigned int robot_id){ // find and return robot who is either causing a collision (either through occupying or is in the process of moving to target cell)
    for(int i = 0; i < tracked_robots.size(); i++){ 
        if(tracked_robots[i].robot_id != robot_id && *movement_cell == tracked_robots[i].robot_position){ // if a collision will occur if robot moves to movement_cell
            return &tracked_robots[i]; // returning pointer to robot information
        }
    }
    return NULL; // if not found, return NULL pointer to notify that no collision has occured
}

void RobotMaster_C::setTargetCellRequest(Coordinates target_cell, unsigned int target_robot){
    Message* messages = new Message(t_Request, -1); // creating new messages for each robot

    RequestHandler* robot_request_handler = getTargetRequestHandler(target_robot); // getting request handler to send response

    // assigning data to message
    m_setTargetCellRequest* message_data = new m_setTargetCellRequest;
    message_data->new_target_cell = target_cell;

    messages->msg_data = message_data; // specifying state to update all robots to

    if(robot_request_handler != NULL){
        robot_request_handler->sendMessage(messages); // sending message
    }
    else{
        delete message_data;
        delete messages;
    }

    return;    
}

bool RobotMaster_C::isRobotMoving(Coordinates C, unsigned int robot_id){

    for(int i = 0; i < tracked_robots.size(); i++){
        if(tracked_robots[i].planned_path.size() > 0 && tracked_robots[i].planned_path[0] == C && tracked_robots[i].robot_moving && tracked_robots[i].robot_id != robot_id){
            return true; // returning true as another robot is in the process of moving to the target cell
        }
    }

    return false;
}