#include "RobotMaster.h"

RobotMaster::RobotMaster(RequestHandler* r){
    id_tracker = 0; // initializing id counter to zero
    Message_Handler = r; // gathering request handler to use for receiving robot -> master communications
}

RobotMaster::~RobotMaster(){
    
}

bool RobotMaster::checkIfOccupied(unsigned int x, unsigned int y, unsigned int* ret_variable){ // checks if a robot is within the cell passed into the function
                                                                                               // returns true is a robot is detected
                                                                                               // if a robot is found, ret_variable is modified to contain the id of the found robot
    for(int i = 0; i < tracked_robots.size(); i++){ // looping through robots position
        if(tracked_robots[i].robot_position.x == x && tracked_robots[i].robot_position.y == y){ // if robot is occupying passed in cell
            *ret_variable = tracked_robots[i].robot_id; // returning found robot id
            return true; // return true as robot is occupying the cell
        }
    }
    return false; // returning false as robot is not found within the occupied cell
}

bool RobotMaster::printGlobalMap(){ // function to print global map of maze including robot location
                                     // maze design based off what can be seen here: https://www.chegg.com/homework-help/questions-and-answers/using-c-1-write-maze-solving-program-following-functionality-note-implementation-details-a-q31826669

    std::string logos[9] = { "   ", "---", "|", " ", " R ", " . ", " X ", " * ", ""}; // array with logos to use when printing maze
    
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

                unsigned int* found_id; // variable to store id of robot if found using checkIfOccupied in the second else if statement

                // checking contents of a node (e.g. does it have a robot?)
                if (j >= maze_xsize){ // if iterating outside of valid node
                    string_pointer = 3; // print empty space as node is outside of maze
                }
                else if(checkIfOccupied(j, i, found_id)){ // if current node is the robot's location
                    printf("%2d ", *found_id); // printing id number of robot within the cell
                                               // TODO: print 3 width digit numbers in centre of cell without error
                    string_pointer = 9; // print nothing after this ifelse statement as the printing has been handled locally
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
                        delete request; // deleting message
                        break;
                    }
                case 1: // updateGlobalMap request
                    {
                    
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
unsigned int RobotMaster::addRobot(unsigned int x, unsigned int y){
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

void RobotMaster::updateGlobalMap(){

}