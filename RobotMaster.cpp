#include "RobotMaster.h"

RobotMaster::RobotMaster(RequestHandler* r){
    id_tracker = 0; // initializing id counter to zero
    Message_Handler = r; // gathering request handler to use for receiving robot -> master communications
    
    maze_xsize = 4;
    maze_ysize = 4;
}

RobotMaster::~RobotMaster(){
    
}

bool RobotMaster::checkIfOccupied(unsigned int x, unsigned int y, unsigned int* ret_variable){ // checks if a robot is within the cell passed into the function
                                                                                               // returns true is a robot is detected
                                                                                               // if a robot is found, ret_variable is modified to contain the id of the found robot
    unsigned int temp = 0;
    for(int i = 0; i < tracked_robots.size(); i++){ // looping through robots position
        if(tracked_robots[i].robot_position.x == x && tracked_robots[i].robot_position.y == y){ // if robot is occupying passed in cell
            temp = tracked_robots[i].robot_id; // returning found robot id
            *ret_variable = temp;
            return true; // return true as robot is occupying the cell
        }
    }
    return false; // returning false as robot is not found within the occupied cell
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

                if(GlobalMap.y_edges[i][j]){ // if there is no edge between two nodes
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
                if(GlobalMap.x_edges[i][j]){ // if there is no edge between two nodes
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
                else if(GlobalMap.nodes[i][j] == 0){ // if current node is invalid (unseen and unexplored)
                    string_pointer = 6; // print I for invalid cell
                }
                else if(GlobalMap.nodes[i][j] == 2){ // if current node has been seen but not explored
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

void RobotMaster::receiveRequests(){
    Message* request = Message_Handler->getMessage(); // gathering request from msg_queue
                                                      // pointer is gathered so response can be gathered by robot threads
                                                      // returned pointer will be NULL is no messages to get
    
    if(request != NULL){ // if there is a a request to handle, process it
        switch (request->request_type){ // determining type of request before processing
            
                case 0: // addRobot request
                    {
                        // addRobot request msg_data layout:
                        // [0] = type: (unsigned int*), content: x coordinate of robot
                        // [1] = type: (unsigned int*), content: y coordinate of robot
                        printf("req1\n");
                        // gathering data from request
                        unsigned int* x = (unsigned int*)request->msg_data[0]; // first pointer of msg_data points to x coordinates
                        unsigned int* y = (unsigned int*)request->msg_data[1]; // second pointer of msg_data points to y coordinates
                        
                        unsigned int robot_id = addRobot(*x, *y); // add robot using coordinates
                                                                  // return value is assigned id of robot

                        request->return_data.push_back((void*)&robot_id); // returning data to sender
                        sem_post(request->response_semaphore);
                        sem_wait(request->ack_semaphore);
                        
                        // sent message and variables were dynamically allocated thus must be deleted as seen below
                        // this part of the code is thread safe as the robot has finished using these variabless
                        sem_destroy(request->ack_semaphore); // deleting semaphores
                        sem_destroy(request->response_semaphore);
                        delete request;
                        break;
                    }
                case 1: // updateGlobalMap request
                    {   
                        printf("req2\n");
                        // updateGlobalMap request msg_data layout:
                        // [0] = type: (unsigned int*), content: id of robot sending request
                        // [1] = type: (vector<bool>*), content: vector containing information on walls surrounding robot
                                                                 // [0] = north, [1] = south, [2] = east, [3] = west
                                                                 // 0 = connection, 1 = wall
                        // [2] = type: (Coordinates*), content: current coordinates of where the read occured
                        unsigned int* robot_id = (unsigned int*)request->msg_data[0];
                        std::vector<bool>* wall_info = (std::vector<bool>*)request->msg_data[1];
                        Coordinates* cords = (Coordinates*)request->msg_data[2];

                        updateGlobalMap(robot_id, wall_info, cords); // updating global map with information

                        //request->return_data.push_back((void*)&robot_id); // telling Robot that data was successfully added to GlobalMap and it can continue
                        int temp;
                        sem_getvalue(request->response_semaphore,&temp);
                        printf("%d semval\n",temp);

                        while(sem_post(request->response_semaphore) != -1) {}
                        
                        sem_getvalue(request->response_semaphore,&temp);
                        printf("%d semval\n",temp);
                        sem_wait(request->ack_semaphore);

                        // sent message and variables were dynamically allocated thus must be deleted as seen below
                        // this part of the code is thread safe as the robot has finished using these variabless
                        sem_destroy(request->ack_semaphore); // deleting semaphores
                        sem_destroy(request->response_semaphore);
                        delete request;

                        break;
                    }
                default:
                    {
                        break;
                    }
            }
    }
    else{ // if no message to handle, do nothing

    }
    
    return;
}

unsigned int RobotMaster::addRobot(unsigned int x, unsigned int y){ // adding robot to control system
                                                                    // this must be completed by all robots before beginning exploration
    
    number_of_unexplored++; // incrementing number of unexplored by 1 as current robot cells has presumably not been explored

    printf("CONTROLLER: adding robot\n");
    id_tracker++; // incrementing inorder to determine next id to give a robot

    RobotInfo temp; // buffer to store robot info before pushing it to the tracked_robots vecto

    temp.robot_id = id_tracker; // assigning id to new robot entry 
    temp.robot_position.x = x;  // assigning position to new robot entry 
    temp.robot_position.y = y;
    temp.robot_status = 0;      // updating current robot status to 0 to leave it on stand by
    
    tracked_robots.push_back(temp); // adding robot info to tracked_robots

    printf("CONTROLLER: returning id = %d\n",temp.robot_id);

    return temp.robot_id; // returning id to be assigned to the robot which triggered this function
}

void RobotMaster::updateGlobalMap(unsigned int* id, std::vector<bool>* connections, Coordinates* C){

    updateRobotLocation(id, C);

    if (GlobalMap.nodes[C->y][C->x] != 1){ // checking if there is a need to update map (has the current node been explored?)
        
        number_of_unexplored--; // subtracting number of unexplored cells as new cell has been explored

        // updating vertical edges in GlobalMap using robot reading
        GlobalMap.y_edges[C->y][C->x] = (*connections)[0]; // north
        GlobalMap.y_edges[C->y + 1][C->x] = (*connections)[1]; // south

        GlobalMap.x_edges[C->y][C->x] = (*connections)[2]; // east
        GlobalMap.x_edges[C->y][C->x + 1] = (*connections)[3]; // west

        GlobalMap.nodes[C->y][C->x] = 1; // updating state of node to be 1 as it has been explored

        // now we will update the neighbouring cells to see if they have previously been explored
        // if not, they will be marked with a '2' on the GlobalMap Nodes Array

        if(!GlobalMap.y_edges[C->y][C->x] && GlobalMap.nodes[C->y - 1][C->x] != 1){ // checking if node to north hasn't been explored by a Robot
            GlobalMap.nodes[C->y - 1][C->x] = 2; // if unexplored and no wall between robot and cell, set northern node to unexplored
            number_of_unexplored++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking south
        if(!GlobalMap.y_edges[C->y + 1][C->x] && GlobalMap.nodes[C->y + 1][C->x] != 1){ // checking if node to north hasn't been explored by a Robot
            GlobalMap.nodes[C->y + 1][C->x] = 2; // if unexplored and no wall between robot and cell, set southern node to unexplored
            number_of_unexplored++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking east
        if(!GlobalMap.x_edges[C->y][C->x] && GlobalMap.nodes[C->y][C->x - 1] != 1){ // checking if node to north hasn't been explored by a Robot
            GlobalMap.nodes[C->y][C->x - 1] = 2; // if unexplored and no wall between robot and cell, set eastern node to unexplored
            number_of_unexplored++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
        // checking wast
        if(!GlobalMap.x_edges[C->y][C->x + 1] && GlobalMap.nodes[C->y][C->x + 1] != 1){ // checking if node to north hasn't been explored by a Robot
            GlobalMap.nodes[C->y][C->x + 1] = 2; // if unexplored and no wall between robot and cell, set western node to unexplored
            number_of_unexplored++; // incrementing number of unexplored nodes by 1 as this neighbouring node has not been explored
        }
    }
    else{ // if there is no need to update map

    }

    printGlobalMap(); // print updated GlobalMap

    return;
}

void RobotMaster::updateRobotLocation(unsigned int* id, Coordinates* C){ // updates the location of a robot to the location specified

    for(int i = 0; i < tracked_robots.size(); i++){
        if (tracked_robots[i].robot_id == *id)
            tracked_robots[i].robot_position = *C;
    }
}
