#include "Robot.h"

Robot::Robot(int x, int y, RequestHandler* outgoing_req, unsigned int xsize, unsigned int ysize){
    // Object Initialization
    x_position = x; // initialising robots current position with passed in values
    y_position = y;

    Robot_2_Master_Message_Handler = outgoing_req; // assigning message handler for robot -> master communications
    Master_2_Robot_Message_Handler = new RequestHandler;

    maze_xsize = xsize; // getting maze size for print out and map allocation purposes
    maze_ysize = ysize;

    LocalMap = new GridGraph(xsize,ysize);

    number_of_unexplored = 1; // set unknown cells to 1 as current occupied cell is unknown to robot

    response_sem = new sem_t; // semaphore to signal from controller to robot that the response message is ready 
    sem_init(response_sem, 0, 0); // initializing semaphore to value of 0. this will cause robot to wait until response is ready
                                  
    acknowledgement_sem = new sem_t; // semaphore to signal from robot to controller that the current response message has been analysed  
    sem_init(acknowledgement_sem, 0, 0); // initializing semaphore to value of 0. this will cause controller to wait until robot is done with response
                    
}

Robot::~Robot(){
    delete Master_2_Robot_Message_Handler; // deleting Master_2_Robot_Message_Handler as no longer needed as object is being destroyed
    delete LocalMap; // deleting LocalMap as dynamically allocated
}

std::vector<bool> Robot::scanCell(GridGraph* maze){ // scans current cell for walls on all sides
                                       // function assumes current cell has not been scanned yet
    std::vector<bool> ret_vector;

    number_of_unexplored--;
    
    // gathering x edges within maze at robot's current position
    LocalMap->x_edges[y_position][x_position] = maze->x_edges[y_position][x_position]; // east
    LocalMap->x_edges[y_position][x_position+1] = maze->x_edges[y_position][x_position+1]; // west
    
    // gathering y edges within maze at robot's current position
    LocalMap->y_edges[y_position][x_position] = maze->y_edges[y_position][x_position]; // north
    LocalMap->y_edges[y_position+1][x_position] = maze->y_edges[y_position+1][x_position]; // south

    // placing edges within return vector for usage by RobotMaster -> [0] = north, [1] = south, [2] = east, [3] = west
    ret_vector.push_back(maze->y_edges[y_position][x_position]); // north
    ret_vector.push_back(maze->y_edges[y_position+1][x_position]); // south
    ret_vector.push_back(maze->x_edges[y_position][x_position]); // east
    ret_vector.push_back(maze->x_edges[y_position][x_position+1]); // west
    

    // updating state of current node 
    LocalMap->nodes[y_position][x_position] = 1; // setting currently scanned node to 1 to signifiy its been scanned 

    // updating state of neighbouring nodes to unexplored if possible
    
    // checking north
    if(!LocalMap->y_edges[y_position][x_position] && LocalMap->nodes[y_position - 1][x_position] != 1){ // checking if node to north hasn't been explored
        LocalMap->nodes[y_position - 1][x_position] = 2; // if unexplored and no wall between robot and cell, set northern node to unexplored
        number_of_unexplored++;
    }
    // checking south
    if(!LocalMap->y_edges[y_position + 1][x_position] && LocalMap->nodes[y_position + 1][x_position] != 1){ // checking if node to north hasn't been explored
        LocalMap->nodes[y_position + 1][x_position] = 2; // if unexplored and no wall between robot and cell, set southern node to unexplored
        number_of_unexplored++;
    }
    // checking east
    if(!LocalMap->x_edges[y_position][x_position] && LocalMap->nodes[y_position][x_position - 1] != 1){ // checking if node to north hasn't been explored
        LocalMap->nodes[y_position][x_position - 1] = 2; // if unexplored and no wall between robot and cell, set eastern node to unexplored
        number_of_unexplored++;
    }
    // checking wast
    if(!LocalMap->x_edges[y_position][x_position + 1] && LocalMap->nodes[y_position][x_position + 1] != 1){ // checking if node to north hasn't been explored
        LocalMap->nodes[y_position][x_position + 1] = 2; // if unexplored and no wall between robot and cell, set western node to unexplored
        number_of_unexplored++;
    }
    return ret_vector;
}

bool Robot::move2Cell(int direction){ // function to move robot depending on location of walls within local map
                                      // ensure current cell is scanned with scanCell before calling this function
                                      // direction = 1 ^ north
                                      // direction = 2 v south
                                      // direction = 3 < east
                                      // direction = 4 > west

    bool ret_value = false; // return variable
                            // used to track whether move operation was a success

    switch (direction){ // switch statement to move robot in specific direction based on know information from local map
    case 1:
        if (!LocalMap->y_edges[y_position][x_position]){ // if there is an edge between current node and node above
            y_position--; // move robot to node above
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 2:
        if (!LocalMap->y_edges[y_position+1][x_position]){ // if there is an edge between current node and node below
            y_position++; // move robot to node below
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 3:
        if (!LocalMap->x_edges[y_position][x_position]){ // if there is an edge between current node and node to the left
            x_position--; // move robot to node to the left
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    case 4:
        if (!LocalMap->x_edges[y_position][x_position+1]){ // if there is an edge between current node and node to the right 
            x_position++; // move robot to node to the right
            ret_value = true; // ret_value = true as movement was a success
        }
        break;

    default: // if invalid direction is passed into function
        break;
    }
    /*if (ret_value) // if robot position movement successfull
        std::cout << "Robot Sucessfully moved to " << x_position  << ", "<< y_position << "\n";*/
    return ret_value; // ret_value = true if robot moved sucessfully
                      // ret_value = false if robot failed to move
}

bool Robot::move2Cell(Coordinates* destination){ // overloaded version of move2Cell using destination
                                                 // converts target destination to a direction then uses original move2Cell function

    // first must determine direction based on two values
    int dx = x_position - destination->x; // gather change in x and y directions
    int dy = y_position - destination->y;

    int direction = 0;  // variable to store determined direction 
                        // direction = 1 ^ north
                        // direction = 2 v south
                        // direction = 3 < east
                        // direction = 4 > west

    if(dy == 1 && dx == 0){ // if moving north
        direction = 1;
    }
    else if(dy == -1 && dx == 0){ // if moving south
        direction = 2;
    }
    else if(dx == 1 && dy == 0){ // if moving east
        direction = 3;
    }
    else if (dx == -1 && dy == 0){ // if moving west
        direction = 4;
    }
    else{
        printf("Error: invalid movement passed into move2Cell\n");
        return false; // return false as movement failed dur to invalid direction
    }

    return move2Cell(direction); // moving robot and returning
    
}

bool Robot::m_move2Cell(Coordinates* destination){ // overloaded version of move2Cell using destination
                                                 // converts target destination to a direction then uses original move2Cell function

    // first must determine direction based on two values
    int dx = x_position - destination->x; // gather change in x and y directions
    int dy = y_position - destination->y;

    int direction = 0;  // variable to store determined direction 
                        // direction = 1 ^ north
                        // direction = 2 v south
                        // direction = 3 < east
                        // direction = 4 > west

    if(dy == 1 && dx == 0){ // if moving north
        direction = 1;
    }
    else if(dy == -1 && dx == 0){ // if moving south
        direction = 2;
    }
    else if(dx == 1 && dy == 0){ // if moving east
        direction = 3;
    }
    else if (dx == -1 && dy == 0){ // if moving west
        direction = 4;
    }
    else{
        printf("Error: invalid movement passed into move2Cell\n");
        return false; // return false as movement failed dur to invalid direction
    }

    // need to message controller to see if target cell is occupied
    Message* temp_message = new Message;

    temp_message->request_type = 2; // request_type = 2 as moveRequest required

    temp_message->msg_data.push_back((void*) &id); // adding id of robot to [0]
    temp_message->msg_data.push_back((void*) destination); // adding target destination of robot to [1]

    temp_message->res_sem = response_sem; // attaching robot communication semaphores to message
    temp_message->ack_sem = acknowledgement_sem;

    Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to robot controller

    sem_wait(response_sem); // waiting for response to be ready from controller

    bool is_not_occupied = (bool)temp_message->return_data[0]; // gather assigned id from RobotMaster's response
    
    sem_post(acknowledgement_sem); // signalling to controller that the response has been utilised 
    
    if(is_not_occupied){ // attempt to move as cell unoccupied
        return move2Cell(direction); // moving robot and returning
    }
    else {
        return false; // return false as target cell occupied
    }
    
}

std::vector<Coordinates> Robot::getValidNeighbours(unsigned int x, unsigned  int y){ // function to gather valid neighbouring cells of a selected cell based on robot's local map

    std::vector<Coordinates> ret_value; // vector of Coordinates to return
                                        // this will contain the coordinates of valid neighbouring nodes
    
    Coordinates buffer; // buffer structor to gather positions of neighbouring nodes before pushing to vector

    // check if neighbour to the north is valid and connected via an edge (no wall)
    if(!LocalMap->y_edges[y][x] && LocalMap->nodes[y-1][x] > 0){
        buffer.x = x; 
        buffer.y = y - 1;
        ret_value.push_back(buffer);
    }
    // check if neighbour to the south is valid and connected via an edge (no wall)
    if(!LocalMap->y_edges[y+1][x] && LocalMap->nodes[y+1][x] > 0){
        buffer.x = x; 
        buffer.y = y + 1;
        ret_value.push_back(buffer);
    }
    // check if neighbour to the east is valid and connected via an edge (no wall)
    if(!LocalMap->x_edges[y][x] && LocalMap->nodes[y][x-1] > 0){
        buffer.x = x - 1; 
        buffer.y = y;
        ret_value.push_back(buffer);
    }
    // check if neighbour to the west is valid and connected via an edge (no wall)
    if(!LocalMap->x_edges[y][x+1] && LocalMap->nodes[y][x+1] > 0){
        buffer.x = x + 1; 
        buffer.y = y;
        ret_value.push_back(buffer);
    }

    /*// printing all valid neighbouring nodes of selected node
    for(int i = 0; i < ret_value.size(); i ++){ // TODO: after debugging
        printf("%d,%d\n",ret_value[i].x,ret_value[i].y);
    }*/

    return ret_value; // returning vector
    // TODO: return by pointer may be desirable
}

bool Robot::pf_BFS(int x_dest, int y_dest){ // function to plan a path for robot to follow from current position to a specified destination
    
    planned_path.clear(); // clearing planned_path as new path is to be planned using breadth first search

    // initial checks to ensure path needs to be planned
    // no point using computation if robot is at destination or destination does not exist on robot's local map
    if(LocalMap->nodes[y_dest][x_dest] == 0) // if the destination node has not been explored
        return false;                       // can't create a path thus return false;

    else if(x_position == x_dest && y_position == y_dest) // if robot is at the destination already
        return true;                                      // no need to move thus return true

    bool ret_value = false; // return value

    std::queue<Coordinates> node_queue; // creating node queue to store nodes to be "explored" by algorithm

    std::map<Coordinates, Coordinates> visited_nodes; // map to track explored nodes to their parent node

    // initializing first node to explore from with robot's current coordinates
    Coordinates curr_node(x_position, y_position); 

    node_queue.push(curr_node); // adding first node to explore to node queue
    visited_nodes.insert({curr_node, curr_node}); // adding first node to visited node maps using itself as parent

    while(node_queue.size() != 0){// while nodes to explore are in node_queue
        
        curr_node = node_queue.front(); // gathering node from front of queue

        //printf("curr node: %d,%d\n", curr_node.x, curr_node.y);

        node_queue.pop(); // removing node from front of the queue

        if (curr_node.x == x_dest && curr_node.y == y_dest){ // if the target node has been located
            ret_value = true; // return true as path found
            break; // break from while loop
        }
        
        std::vector<Coordinates> valid_neighbours = getValidNeighbours(curr_node.x, curr_node.y); // gathering neighbours of current node

        for(int i = 0; i < valid_neighbours.size(); i ++){ // iterate through all of the current node's neighbours to see if they have been explored

            if(visited_nodes.contains(valid_neighbours[i]) == 0){ // if current neighbour has not been visited
                node_queue.push(valid_neighbours[i]);       // TODO: fix map.contains, still causes same error where not detecting key         
                visited_nodes.insert({valid_neighbours[i], curr_node});   
            }
        }
    }
    
    // TODO: implement handling if path to target location is not found 

    // as a valid path has been found from current position to target using robot's local map
    // must travel from destination back through parent nodes to reconstruct path
    
    curr_node.x = x_dest; // ensuring the current node is pointing to destination
    curr_node.y = y_dest; // this allows for the "walk back" through nodes to start to occur
    
    while(!(curr_node.x == x_position && curr_node.y == y_position)){ // while the starting node has not been found from parents

        planned_path.push_back(curr_node); // add current node to planned path

        for(auto [key, val]: visited_nodes){ // TODO: use better search for key function
            if (key == curr_node){
                curr_node = val;
                break;
            }
        }
    }

    /*printf("Planned Path\n"); // printing planned path
    for(int i = 0; i <  planned_path.size(); i++){ // TODO: Remove after further debugging
        printf("%d,%d\n",planned_path[i].x,planned_path[i].y);
    }*/

    return ret_value; 
}

bool Robot::BFS_pf2NearestUnknownCell(std::vector<Coordinates>* ret_vector){

    bool ret_value = false; // return value

    std::queue<Coordinates> node_queue; // creating node queue to store nodes to be "explored" by algorithm

    std::map<Coordinates, Coordinates> visited_nodes; // map to track explored nodes to their parent node

    // initializing first node to explore from with robot's current coordinates
    Coordinates curr_node(x_position, y_position); 

    node_queue.push(curr_node); // adding first node to explore to node queue
    visited_nodes.insert({curr_node, curr_node}); // adding first node to visited node maps using itself as parent

    while(node_queue.size() != 0){// while nodes to explore are in node_queue
        
        curr_node = node_queue.front(); // gathering node from front of queue

        //printf("curr node: %d,%d\n", curr_node.x, curr_node.y);

        if (LocalMap->nodes[curr_node.y][curr_node.x] == 2){ // if an unexplored node has been found
            ret_value = true; // return true as path to unexplored node found
            break; // break from while loop
        }
        else if(node_queue.size() < 1){ // if node_queue is empty, no unexplored nodes found 
                                        // true can be returned as no errors occured in pathfinding
            return true;                // ret_vector will be empty as no unexplored nodes found therefore no need to move
        }

        node_queue.pop(); // removing node from front of the queue as new nodes must be added to queue

        std::vector<Coordinates> valid_neighbours = getValidNeighbours(curr_node.x, curr_node.y); // gathering neighbours of current node

        for(int i = 0; i < valid_neighbours.size(); i ++){ // iterate through all of the current node's neighbours to see if they have been explored
            if(visited_nodes.contains(valid_neighbours[i]) == 0){ // TODO: fix map.contains, still causes same error where not detecting key
                node_queue.push(valid_neighbours[i]);              
                visited_nodes.insert({valid_neighbours[i], curr_node});   
            }
        }
    }
    
    // TODO: implement handling if path unexplored cell not found 

    if (ret_value == false){ // if ret_value == fsle, 

    }

    // as a valid path has been found from current position to target using robot's local map
    // must travel from destination back through parent nodes to reconstruct path

    ret_vector->clear(); // clearing planned_path as new path is to be planned using breadth first search

    while(!(curr_node.x == x_position && curr_node.y == y_position)){ // while the starting node has not been found from parents

        ret_vector->push_back(curr_node); // add current node to planned path

        for(auto [key, val]: visited_nodes){ // TODO: use better search for key function
            if (key == curr_node){
                curr_node = val;
                break;
            }
        }
    }

    /*printf("Planned Path\n"); // printing planned path
    for(int i = 0; i <  ret_vector->size(); i++){ // TODO: Remove after further debugging
        printf("%d,%d\n",(*ret_vector)[i].x,(*ret_vector)[i].y);
    }*/

    return ret_value; 
}

void Robot::soloExplore(GridGraph* maze){
    while(1){

        scanCell(maze); // scan cell 
        
        if(number_of_unexplored == 0){ // no more cells to explore therefore break from scan loop
            printf("Done exploring!\n");
            break;
        }

        printRobotMaze(); // print maze contents for analysis

        BFS_pf2NearestUnknownCell(&planned_path); // move to nearest unseen cell

        for (int i = 0; i < planned_path.size(); i++){ // while there are movements left to be done by robot
            move2Cell(&(planned_path[planned_path.size()-i-1]));
        }
        planned_path.clear(); // clear planned_path as movement has been completed
    }

    return;
}

void Robot::multiRobotLoop(GridGraph* maze){ // function to initialize robot before it starts operating

    assignIdFromMaster(); // getting id from robotmaster before begining robot exploration
    int status = 0; // tracks status of robot
                    // -1 = shut down
                    // 0 = stand by
                    // 1 = explore
    while(status != -1){
        
        status = getRequestsFromMaster(status); // checking if master wants robot to update status

        switch(status){

            case 0: // while robot is on standby
                {   
                    // do nothing
                    break;
                }
            case 1: // while on explore mode
                {   
                    // continuously explore until master changes operation
                    multiExplore(maze); // do one cycle of exploration
                    break;
                }
        }        
    }
    return;
}

void Robot::multiExplore(GridGraph* maze){

    std::vector<bool> connection_data = scanCell(maze); // scan cell which is occupied by the robot 
    
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

 //   do{
        
        BFS_pf2NearestUnknownCell(&planned_path); // create planned path to nearest unknown cell

  /*      Message* temp_message = new Message(); // buffer to load data into before sending message

        temp_message->request_type = 1; // request_type = 0 as updateGlobalMap request required

        temp_message->msg_data.push_back((void*) &id); // adding id of robot sending request to [0]
        temp_message->msg_data.push_back((void*) &connection_data); // adding vector containing information on walls surrounding robot to [1]
        temp_message->msg_data.push_back((void*) &robot_cords); // current coordinates of where the read occured
        
        temp_message->res_sem = response_sem; // attaching communication semaphores to message
        temp_message->ack_sem = acknowledgement_sem;
        
        Robot_2_Master_Message_Handler->sendMessage(temp_message); // sending message to message queue

        sem_wait(response_sem); // waiting for data to be inputted into Master before continuing
        sem_post(acknowledgement_sem);

    }while();*/

    for (int i = 0; i < planned_path.size(); i++){ // while there are movements left to be done by robot
        if(!m_move2Cell(&(planned_path[planned_path.size()-i-1]))) // if movement fails
            break; // break from loop
    }
    planned_path.clear(); // clear planned_path as movement has been completed

    return;
}

void Robot::assignIdFromMaster(){

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


int Robot::getRequestsFromMaster(int status){ // checking if RobotMaster wants robot to change states
    
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
                        
                        sem_post(request->res_sem); // telling RobotMaster that status has sucessfully been updated
                        break;
                    }
                
                case 0: // change state to standby mode
                    {
                        //ret_variable = request->request_type;
                        //sem_post(request->res_sem);
                        
                        break;
                    }
                case 1: // change state to begin exploring
                    {
                        printf("ROBOT %d: Begin Exploring\n", id);
                        // if a message of this type is recieved, no message contents
                        // only the response semaphore is used to signal that the robot will begin exploring
                        ret_variable = request->request_type; // gathering status from request type
                        
                        sem_post(request->res_sem); // telling RobotMaster that status has sucessfully been updated
                        break;
                    }
                default: // TODO: Implement handling if improper messge type is recieved
                    {
                        break;
                    }
            }
    }
    else{ // if no message to handle, do nothing
        ret_variable = status;
    }
    
    return ret_variable;
}

bool Robot::printRobotMaze(){ // function to print robot's local map of maze
                              // maze design based off what can be seen here: https://www.chegg.com/homework-help/questions-and-answers/using-c-1-write-maze-solving-program-following-functionality-note-implementation-details-a-q31826669

    std::string logos[8] = { "   ", "---", "|", " ", " R ", " . ", " X ", " * "}; // array with logos to use when printing maze
    
    if(maze_xsize == 0 || maze_ysize == 0){ // if maze has not been allocated
        printf("Error: Maze size has not been specified\n");
        return false; // return false as printing failed
    }

    printf("*Robot Local Map**\n"); // printing title and maze information

    int string_pointer = 0; // integer used to determine which logo needs to be printed from logo vector

    int count = 0; // counter to determine if both the column and row edges have been printed
    int i = 0; // counter to track if the whole maze has been printed

    while(!(i >= maze_ysize && count == 1)){ // while loop to determing which row to print (i = node row number)
    
        if(count == 0){ // printing the horizontal walls of maze

            for(int j = 0; j < maze_xsize; j++){

                if(LocalMap->y_edges[i][j]){ // if there is no edge between two nodes
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
                if(LocalMap->x_edges[i][j]){ // if there is no edge between two nodes
                    string_pointer = 2; // print horizontal line
                }
                else{ // if there is an edge between two nodes
                    string_pointer = 3; // print horizontal line
                }
                printf("%s", logos[string_pointer].c_str());

                // checking contents of a node (e.g. does it have a robot?)
                if (j >= maze_xsize){ // if iterating outside of valid node
                    string_pointer = 3; // print empty space as node is outside of maze
                }
                else if(j == x_position && i == y_position){ // if current node is the robot's location
                    string_pointer = 4; // print R for robot
                }
                else if(LocalMap->nodes[i][j] == 0){ // if current node is invalid (unseen and unexplored)
                    string_pointer = 6; // print I for invalid cell
                }
                else if(LocalMap->nodes[i][j] == 2){ // if current node has been seen but not explored
                    string_pointer = 7; // print * for seen node
                }
                else{ // if current cell has been seen and explored (valid)
                    string_pointer = 0; // print empty space
                }


                printf("%s",logos[string_pointer].c_str());
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

void Robot::printRobotNodes(){ // function to print Robot's map of explored nodes
    printNodes(LocalMap);
}

void Robot::printRobotXMap(){ // function to print Robot's X edge map 
    printXEdges(LocalMap);
}
void Robot::printRobotYMap(){ // function to print Robot's Y edge map 
    printYEdges(LocalMap);
}