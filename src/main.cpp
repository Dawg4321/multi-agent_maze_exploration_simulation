#include <iostream>
#include <fstream>
#include <pthread.h>
#include <filesystem>
#include <utility>
#include <random>

#include "Maze.h"
#include "RobotMaster_NC.h"
#include "RobotMaster_NC_Greedy.h"
#include "RobotMaster_C_Greedy.h"
#include "RobotMaster_C_FCFS.h"
#include "MultiRobot_NC.h"
#include "MultiRobot_NC_CellReservation.h"
#include "MultiRobot_C_CellReservation.h"

using namespace std;

using json = nlohmann::json; // simplifying namespace so "json" can be used instead of "nlohmann::json" when declaring json objects

struct TurnControlData{ // struct containing data used by robot and master threads to control turn flow
    
    pthread_barrier_t turn_start_barrier, turn_end_barrier; // barriers to synchronize robot threads into a turn format
    
    pthread_mutex_t finished_counter_mutex; // mutexes to protect the counters

    bool all_robots_completed; // tracks number of robots which have completed a turn

    TurnControlData(int num_robots){
        // initialzing barrier to number of robots + 1 (all robots + robot master must be waiting before next turn can start/finish)
        pthread_barrier_init(&turn_start_barrier, NULL, num_robots + 1);
        pthread_barrier_init(&turn_end_barrier, NULL, num_robots + 1);

        // intialzing pthread mutexes
        pthread_mutex_init(&finished_counter_mutex, NULL);

        all_robots_completed = false; // initializing finished_robots to 0 as no robots have finished executing fully
    }
    ~TurnControlData(){
        // detroying pthread barrier and mutexes
        pthread_barrier_destroy(&turn_start_barrier);
        pthread_mutex_destroy(&finished_counter_mutex);
    }
};

struct RobotMasterArgs{ // structure to hold args for passing RobotMaster information into a new thread
    RobotMaster* Generated_RobotMaster; // dynamically allocated robotmaster

    TurnControlData* turn_control; // struct containing info to control robot's turn

    json turn_json; // json containing request infomation of robot's action during each turn
    
    vector<string> maze_printouts; // vector containing maze printouts for export to text files

    RobotMasterArgs(RobotMaster* R1, TurnControlData* control_info){
        Generated_RobotMaster = R1;
        turn_control = control_info;
    }
};

struct RobotArgs{ // structure to hold args for passing robot information into a new thread
    MultiRobot* Generated_Robot; // dynamically allocated robot
    GridGraph Maze_Map; // Map of maze used by robot to scan cells

    TurnControlData* turn_control; // struct containing info to control robot's turn

    RobotArgs(MultiRobot* R1, GridGraph M, TurnControlData* control_info){
        Generated_Robot = R1;
        Maze_Map.nodes = M.nodes;
        Maze_Map.x_edges = M.x_edges;
        Maze_Map.y_edges = M.y_edges;
        turn_control = control_info;
    }
};

int getTurns2Wait(int last_status_of_execution){ // returns the number of turns to wait depending on type of request processed

    switch(last_status_of_execution){
        case s_compute_move: // if robot is currently moving
        {
            return 3; // wait a four turns before update master of new position
        }
        case s_scan_cell: // if robot is currently scanning a cell
        {
            return 2; // wait a turn to before sending scan data to master
        }
        default: // if robot performing any other operation
        {
            return 1; // don't wait any turns
        }
    }
}

void* robotFunc(void* Robot_Info){ // function for robot running threads
    // gathering passed data
    RobotArgs* Data = (RobotArgs*) Robot_Info; // argument structure containing passed data
    MultiRobot* R = Data->Generated_Robot; // robot to run in this thread
    TurnControlData* TurnControl = Data->turn_control; // turn control mechanisms

    R->robotSetUp(); // setting up robot before loop intialization

    int robot_execution_status = s_stand_by; // tracks what state the robot executed in last robot loop pass
    
    int number_of_turns_to_wait = 0; // tracks the number of turns robot has to sit out before executing a robotloop step
                                     // initialzing to zero as robot should not wait on its first turn
    
    while(robot_execution_status != s_exit_loop){ // continue executing turns until the robot has been put into shut down state

        pthread_barrier_wait(&TurnControl->turn_start_barrier); // waiting for all threads to complete preivous turn initialization before starting next turn

        if(number_of_turns_to_wait == 0){ // if robot does not have to sit out for a turn, execute robot loop step

            robot_execution_status = R->robotLoopStepforSimulation(&Data->Maze_Map); // executing one step of the robot loop
 
            number_of_turns_to_wait = getTurns2Wait(robot_execution_status); // determing how many turns robot has to sit out for before next operation 
                                                                             // these turns help give the illusion of time taken for each type of request
        }
        else if(number_of_turns_to_wait == 1){ // if there is only 1 turn left to wait, compute the function which has been waiting
            if(robot_execution_status == s_scan_cell){
                R->computeScanCell(&Data->Maze_Map);
            }
            else if(robot_execution_status == s_compute_move){
                R->computeMove();
            }
        }

        pthread_barrier_wait(&TurnControl->turn_end_barrier); // waiting for all threads to complete preivous turn initialization before starting next turn
        
        number_of_turns_to_wait--; // subtract number of turns to wait as robot has finished a turn
    }

    bool all_robots_done = false; // boolean to track whether all robots have completed their turns

    pthread_mutex_lock(&TurnControl->finished_counter_mutex);
    if(TurnControl->all_robots_completed){ // checking if all robots are completed (e.g. is this the last robot to finish)
           all_robots_done = true; // no need to utilise barrier to facilitate more turns
        }
    pthread_mutex_unlock(&TurnControl->finished_counter_mutex);

    while(!all_robots_done){ // loop to allow other robots to complete their turns by using the barriers

        pthread_barrier_wait(&TurnControl->turn_start_barrier); // using various turn control barrier to allow other robots to complete their turns
        pthread_barrier_wait(&TurnControl->turn_end_barrier);

        // checking if all robots are completed (e.g. does this thread need to keep using the barriers to ensure other threads finish)
        pthread_mutex_lock(&TurnControl->finished_counter_mutex);
        if(TurnControl->all_robots_completed){ // all robots have completed their turns 
           all_robots_done = true; // exit loop as no need to use various barriers anymore
        }
        pthread_mutex_unlock(&TurnControl->finished_counter_mutex);

    }

    pthread_exit(NULL); // return from thread
}

void* controllerFunc(void* RobotMaster_Info){ // function to run Robot Controller in a seperate thread
    // gathering passed data
    RobotMasterArgs* Data = (RobotMasterArgs*) RobotMaster_Info; // argument structure containing passed data
    RobotMaster* RM = Data->Generated_RobotMaster; // robot master to run
    TurnControlData* TurnControl = Data->turn_control; // turn control mechanisms

    unsigned int turn_counter = 0; // counter to track number of turns which have occured

    RM->robotMasterSetUp(); // setting up robot master before receiving requests

    bool maze_mapped = false;

    pthread_barrier_wait(&TurnControl->turn_start_barrier); // start robot's first turn

    while(!maze_mapped){ // RobotMaster loop
        
        pthread_barrier_wait(&TurnControl->turn_end_barrier); // waiting for robots to finish turn

        turn_counter++; // incrementing turn counter as a turn has finished

        while(RM->getNumRequestsinQueue() != 0){ // while there are requests to receive on this turn, handle them
            maze_mapped = RM->receiveRequests();
        }
        
        json buffer_json; // load requests handled during turn into a json
        
        string name = "Turn_"; // creating turn number name
        name += to_string(turn_counter);
        
        buffer_json[name] = RM->getRequestInfo(); // gathering json containing request info during this turn
        Data->turn_json["Simulation"].push_back(buffer_json);
        RM->clearRequestInfo(); // clearing contents of request info before next turn

        if(!maze_mapped){ // if statement to prevent maze from being print once all robots have completed exploration
            cout << "*Turn_" << turn_counter << "*\n"; // printing turn number
            Data->maze_printouts.push_back("*Turn_" + to_string(turn_counter) + "*\n" + RM->printGlobalMap()); // printing global map
        }

        pthread_barrier_wait(&TurnControl->turn_start_barrier); // signalling robots to begin next turn
    }

    // safely telling other threads that all robots have completed their exploration
    pthread_mutex_lock(&TurnControl->finished_counter_mutex);
    TurnControl->all_robots_completed = true;
    pthread_mutex_unlock(&TurnControl->finished_counter_mutex);

    pthread_barrier_wait(&TurnControl->turn_end_barrier); // Signalling robot they can finally exit their loop after completion
    
    Data->turn_json["Info"]["Total_Turns_Taken"] = turn_counter;
    Data->turn_json["Info"]["Number_of_Robots"] = RM->getNumberofRobots();
    Data->turn_json["Maze_Characteristics"] = { {"X_Size", }, {"Y_Size", }, {"Node_Map", }, {"X_Edges", }, {"Y_Edges"}};
    Data->turn_json["Info"]["Number_of_Printouts"] = Data->maze_printouts.size(); // adding number of printouts to simulation.json

    pthread_exit(NULL); // return from thread
}

MultiRobot* getNewRobot(int robot_type, int x_pos, int y_pos, RequestHandler* request_handler, unsigned int xsize, unsigned int ysize){
    
    switch(robot_type){ // returning selected robot type
        case 1: // Selecting No Collision, Unintelligent Exploration
        {
            return new MultiRobot_NC(x_pos, y_pos, request_handler, xsize, ysize);
        }
        case 2: // Selecting No Collision, Intelligent Exploration
        {
            return new MultiRobot_NC_CellReservation(x_pos, y_pos, request_handler, xsize, ysize);
        }
        case 3:
        case 4:
        {
            return new MultiRobot_C_IE(x_pos, y_pos, request_handler, xsize, ysize);
        }
    }
}

RobotMaster* getNewRobotMaster(int robot_type, int number_of_robots, RequestHandler* request_handler, unsigned int xsize, unsigned int ysize){
    
    if(robot_type == 1){ // if the robots to simulate are of type NC_UI
        return new RobotMaster_NC(request_handler, number_of_robots, xsize, ysize);
    }
    else if(robot_type == 2){ // if the robots to simulate are of type NC_IE
        return new RobotMaster_NC_Greedy(request_handler, number_of_robots, xsize, ysize);
    }
    else if(robot_type == 3){ // if the robots to simulate are of type C_IE
        return new RobotMaster_C_Greedy(request_handler, number_of_robots, xsize, ysize);
    }
    else if(robot_type == 4){
        return new RobotMaster_C_FCFS(request_handler, number_of_robots, xsize, ysize);
    }
}

bool exportJSON(json json_2_export, string json_name, string target_directory){


    json_name = target_directory + json_name + ".json"; // adding target_directory and .json extension to passed in name

    std::ofstream json_file(json_name); // creating file stream to the json file

    if(json_file.is_open()){ // if file was created successfully

        json_file << std::setw(4) << json_2_export << std::endl; // export json to .json file

        return true;
    }

    // failed to open file for export

    cout << "Error: Failed to write to " << json_name;

    return false;
}

void exportPrintOuts(vector<string>* strings_to_export, string target_directory){

    std::filesystem::create_directories(target_directory + "printouts"); // creating directory to store printouts in  

    for(int i = 0; i < (*strings_to_export).size(); i++){
        string print_name = target_directory + "printouts/" + "printout_" + to_string(i+1) + ".txt"; // creating file in target_directory

        std::ofstream json_file(print_name); // creating file stream to the printouts to      
        if(json_file.is_open()){ // if file was created successfully
            json_file << (*strings_to_export)[i]; // export printout to txt file
        }
    }

    return;
}

void simulateOneTime(){
    // ~~~ Maze Selection ~~~
    cout << "Which Maze would you like to simulate?\n";
    cout << "1 - 4x4 Sample Maze\n";
    cout << "2 - NxN empty grid\n";

    int maze_selection_input; // variable to store input

    Maze Generated_Maze; // Maze object to be used for maze allocation
    cin >> maze_selection_input; // gathering input value

    switch(maze_selection_input){ // returning selected robot type
        case 1: // 4x4 Sample Maze
        {
            Generated_Maze.generate4x4SampleMaze(); // generating sample maze
            Generated_Maze.printMaze();

            break;
        }
        case 2: // NxN empty grid
        {
            // determining grid size
            int x_size, y_size;

            // x position
            cout << "Maze x size: "; 
            cin >> x_size;
            // y position
            cout << "Maze y size: ";
            cin >> y_size;

            Generated_Maze.generateInterconnectedMaze(x_size, y_size); // generating NxN empty grid  
        
            break;
        }
        case 3:
        {
            Generated_Maze.generate8x8SampleMaze(); // generating sample maze
            Generated_Maze.printMaze();

            break;
        }
        case 4:
        {
            Generated_Maze.generateRandomNxNMaze(50,50);
            Generated_Maze.printMaze();
            
            break;
        }
    }

    // ~~~ Gathering Robot Simulation Information ~~~

    // determining number of robots to simulate
    cout << "How many robots would you like to simulate?\n";
    
    int number_of_robots = 0;
    cin >> number_of_robots;

    // determining type of robots to simulate
    cout << "What type of robots do you want to simulate?\n";
    cout << "1 - No Collision, Unintelligent Exploration\n";
    cout << "2 - No Collision, Intelligent Exploration\n";
    cout << "3 - Collision, Intelligent Exploration\n";

    int type_of_robots = 0;
    cin >> type_of_robots;

    // ~~~ Turn Tracking System Variable Creation ~~~~
    TurnControlData turn_control_data(number_of_robots);
    
    // ~~~ Robot Master Thread Generation ~~~
    RequestHandler* request_handler = new RequestHandler(); // creating message handler for robot -> master communcation
    
    // gathering new RobotMaster compatible with specified type of robots
    RobotMaster* Robot_Controller = getNewRobotMaster(type_of_robots, number_of_robots, request_handler, Generated_Maze.getMazeXSize(), Generated_Maze.getMazeYSize());
    RobotMasterArgs RMArgs(Robot_Controller, &turn_control_data);

    // creating thread to run Robot_Controller 
    pthread_t master_thread;
    pthread_create(&master_thread, NULL, &controllerFunc, (void*)&RMArgs);

    // ~~~ Robot Thread Generation ~~~
    MultiRobot* Robots_Array[number_of_robots]; // generating array for robots to be stored in
    RobotArgs* Robot_Thread_Args[number_of_robots]; // generating array for arguments to be passed into robot threads

    pthread_t thread_id[number_of_robots]; // creating threads for each robot              
    
    for (int i = 0; i < number_of_robots; i++){
        // determining robot position within maze
        int robot_x_position, robot_y_position;

        // x position
        cout << "Robot " << i <<". set x position:"; 
        cin >> robot_x_position;
        // y position
        cout <<"Robot " << i <<". set y position:";
        cin >> robot_y_position;

        Robots_Array[i] = getNewRobot(type_of_robots, robot_x_position, robot_y_position, request_handler, Generated_Maze.getMazeXSize(), Generated_Maze.getMazeYSize());
        
        // passing robot into
        Robot_Thread_Args[i] = new RobotArgs(Robots_Array[i], Generated_Maze.getMazeMap(), &turn_control_data);
        // running robot thread
        pthread_create(&thread_id[i], NULL, &robotFunc, (void*)Robot_Thread_Args[i]);
    }

    // ~~~ Awaiting Robot Thread Completion  ~~~
    for(int i = 0; i < number_of_robots; i++) // waiting for robot threads to finish
        pthread_join(thread_id[i], NULL); 

    // ~~~ Awaiting Robot Master Thread Completion ~~~
    pthread_join(master_thread, NULL); // waiting for robot master thread to finish
    
    exportJSON(RMArgs.turn_json, "Simulation", "./");

    // ~~~ Deleting Dynamically Allocated Memory and Barriers ~~~

    delete Robot_Controller; // deleting RobotMaster
    
    for(int i = 0; i < number_of_robots; i++) // deleting all generated robots
        delete Robots_Array[i];

    delete request_handler; // deleting request handler used by robots

    return;
}

void testCases(){

    int num_robots = 3; // two robots

    RequestHandler* req = new RequestHandler;

    TurnControlData turn_control_data(num_robots);

    GridGraph g1; // gridgraph for maze data

    g1.nodes = {{1, 1, 1, 1, 1, 1, 1, 1, 1},
                {0, 0, 0, 0, 0, 0, 0, 1, 0},
                {0, 0, 0, 0, 0, 0, 0, 1, 0}};

    g1.x_edges = {{1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                  {0, 0, 0, 0, 0, 0, 0, 1, 1, 0},
                  {0, 0, 0, 0, 0, 0, 0, 1, 1, 0}};

    g1.y_edges = {{1, 1, 1, 1, 1, 1, 1, 1, 1},
                  {1, 1, 1, 1, 1, 1, 1, 0, 1},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 1, 0}};
    
    GridGraph g2; // gridgraph for Local/Global Maps
    
    g2.nodes = {{2, 1, 1, 1, 1, 1, 1, 2, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0}};

    g2.x_edges = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

    g2.y_edges = {{0, 1, 1, 1, 1, 1, 1, 0, 0},
                  {0, 1, 1, 1, 1, 1, 1, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0, 0, 0, 0}};
                
    // initialzing robot positions
    RobotMaster* RM1 = new RobotMaster_NC_Greedy(req, num_robots, 9, 3);
    RobotMasterArgs RM1args(RM1, &turn_control_data);
    RM1->setGlobalMap(&g2);
    
    MultiRobot* R1 = new MultiRobot_NC_CellReservation(4, 0, req, 9, 3);
    RobotArgs R1args(R1, g1, &turn_control_data);
    R1->setLocalMap(&g2);
    MultiRobot* R2 = new MultiRobot_NC_CellReservation(5, 0, req, 9, 3);
    RobotArgs R2args(R2, g1, &turn_control_data);
    R2->setLocalMap(&g2);
    MultiRobot* R3 = new MultiRobot_NC_CellReservation(6, 0, req, 9, 3);
    RobotArgs R3args(R3, g1, &turn_control_data);
    R3->setLocalMap(&g2);

    // creating threads
    pthread_t T1, T2, T3, T4;

    pthread_create(&T1, NULL, &controllerFunc, (void*)&RM1args);
    pthread_create(&T2, NULL, &robotFunc, (void*)&R1args);
    pthread_create(&T3, NULL, &robotFunc, (void*)&R2args);
    pthread_create(&T4, NULL, &robotFunc, (void*)&R3args);

    // awaiting threads 
    pthread_join(T1, NULL); 
    pthread_join(T2, NULL); 
    pthread_join(T3, NULL); 
    pthread_join(T4, NULL); 

    // exporting JSON
    exportJSON(RM1args.turn_json, "Test_Case", "./");

    // deleting dynamically allocated data
    delete RM1;
    delete R1;
    delete R2;
    delete R3;
    delete req;

    return;
}


void runSimulation(Maze* Generated_Maze, int number_of_robots, int type_of_robots, vector<Coordinates>* robot_start_positions, string export_target_directory){
    // ~~~ Turn Tracking System Variable Creation ~~~~
    TurnControlData turn_control_data(number_of_robots);
    
    // ~~~ Robot Master Thread Generation ~~~
    RequestHandler* request_handler = new RequestHandler(); // creating message handler for robot -> master communcation
    
    // gathering new RobotMaster compatible with specified type of robots
    RobotMaster* Robot_Controller = getNewRobotMaster(type_of_robots, number_of_robots, request_handler, Generated_Maze->getMazeXSize(), Generated_Maze->getMazeYSize());
    RobotMasterArgs RMArgs(Robot_Controller, &turn_control_data);

    // creating thread to run Robot_Controller 
    pthread_t master_thread;
    pthread_create(&master_thread, NULL, &controllerFunc, (void*)&RMArgs);

    // ~~~ Robot Thread Generation ~~~
    MultiRobot* Robots_Array[number_of_robots]; // generating array for robots to be stored in
    RobotArgs* Robot_Thread_Args[number_of_robots]; // generating array for arguments to be passed into robot threads

    pthread_t thread_id[number_of_robots]; // creating threads for each robot              
    
    for (int i = 0; i < number_of_robots; i++){

        Robots_Array[i] = getNewRobot(type_of_robots, (*robot_start_positions)[i].x, (*robot_start_positions)[i].y, request_handler, Generated_Maze->getMazeXSize(), Generated_Maze->getMazeYSize()); // gathering new robot of specified type and start position
        
        // passing robot into
        Robot_Thread_Args[i] = new RobotArgs(Robots_Array[i], Generated_Maze->getMazeMap(), &turn_control_data);
        // running robot thread
        pthread_create(&thread_id[i], NULL, &robotFunc, (void*)Robot_Thread_Args[i]);
    }

    // ~~~ Awaiting Robot Thread Completion  ~~~
    for(int i = 0; i < number_of_robots; i++) // waiting for robot threads to finish
        pthread_join(thread_id[i], NULL); 

    // ~~~ Awaiting Robot Master Thread Completion ~~~
    pthread_join(master_thread, NULL); // waiting for robot master thread to finish
    
    exportJSON(RMArgs.turn_json, "Simulation", export_target_directory); // exporting json
    exportPrintOuts(&RMArgs.maze_printouts, export_target_directory); // exporting print outs

    // ~~~ Deleting Dynamically Allocated Memory and Barriers ~~~

    delete Robot_Controller; // deleting RobotMaster
    
    for(int i = 0; i < number_of_robots; i++) // deleting all generated robots
        delete Robots_Array[i];

    delete request_handler; // deleting request handler used by robots

    return;
}

void testGroupSize(){
    
    int maze_size;
    cout << "What Size Maze to Perform Tests on?\n";
    cin >> maze_size;

    int number_of_mazes;
    cout << "How many Mazes to generate for tests?\n";
    cin >> number_of_mazes;

    int number_of_robots;
    cout << "How many robots to simulate?\n";
    cin >> number_of_robots;

    int type_of_robot;
    cout << "What type of robot to use?\n";
    cin >> type_of_robot;
    
    // getting all factor pairs of the number of robots
    vector<pair<int,int>> group_sizes; // first = number of robots in group, second = number of start positions
    for(int i = 1; i < number_of_robots + 1; i++){
        if(number_of_robots % i == 0){ // if the number is a factor
            group_sizes.push_back(pair<int,int>(i, number_of_robots/i)); // add factor and its pair to group_sizes
        }
    }

    string target_directory;
    cout << "Enter a directory to store results of the simulation:\n";
    cin >> target_directory;

    json simulation_info;
    simulation_info["Number_of_Robots"] = number_of_robots;
    // creating directories for simulation
    for(int i = 0; i < group_sizes.size(); i++){
        std::filesystem::create_directories(target_directory + to_string(number_of_robots) + "_group_size_" + to_string(group_sizes[i].first)); // creating parent directories to store robot simulations of various swarm sizes
        simulation_info["Group_Sizes"].push_back(group_sizes[i].first); // adding group size to simulation_info json 
    }

    for(int j = 0; j < number_of_mazes; j++){

        for(int i = 0; i < group_sizes.size(); i++){

            vector<Coordinates> start_positions; // vector to store start positions for each robot
            
            std::random_device rd; // non-deterministic number generator
            std::mt19937 rand_location(rd()); // seeding mersenne twister
            std::uniform_int_distribution<> dist(1, 2*maze_size + 2*(maze_size - 2)); // distribute random value between 1 and the number of border cells

            for(int group_num = 0; group_num < group_sizes[i].second; group_num++){
                
                bool valid_location = false; // boolean to track whether the group's start location is valid
            
                while(!valid_location){
                    int chosen_location = dist(rand_location); // choosing a random border cell
                    Coordinates chosen_start_position;

                    int xpos = 0;
                    int ypos = 0;
                    bool cell_found = false;
                    while(!cell_found){
                        
                        chosen_location--;
                        if(chosen_location == 0){
                            cell_found = true;
                            chosen_start_position = Coordinates(xpos,ypos);
                        }
                        else{
                            if(ypos == 0 && xpos < maze_size - 1){
                                xpos++;
                            }
                            else if(xpos == maze_size - 1 && ypos < maze_size - 1){
                                ypos++;
                            }
                            else if(ypos == maze_size - 1 && xpos > 0){
                                xpos--;
                            }
                            else{
                                ypos--;
                            }
                        }
                    }

                    bool position_already_in_use = false;
                    for(int k = 0; k < start_positions.size(); k++){
                        if(chosen_start_position == start_positions[k]){
                            position_already_in_use = true;
                        }
                    }

                    if(!position_already_in_use){
                        valid_location = true; // setting valid_location to true as a new group start position has been found
                        for(int k = 0; k < group_sizes[i].first; k++) // adding start positions according to the number of robots in a group
                            start_positions.push_back(chosen_start_position);
                    }
                }
            }
            

            Maze m;
            m.generateRandomNxNMaze(maze_size, maze_size); // generating new random maze

            string directory_for_export = target_directory + to_string(number_of_robots) + "_group_size_" + to_string(group_sizes[i].first) + "/sim_" + to_string(j + 1) +  "/"; // getting directory for target output
            std::filesystem::create_directories(directory_for_export); // creating child directory to store robot simulation for this test

            runSimulation(&m, number_of_robots, 3, &start_positions, directory_for_export); // running simulation
        }
    }

    exportJSON(simulation_info, "Sim_Settings", target_directory);

    return;
}

void testSwarmSize(){
    
    int maze_size;
    cout << "What Size Maze to Perform Tests on?\n";
    cin >> maze_size;

    int number_of_mazes;
    cout << "How many Mazes to generate for tests?\n";
    cin >> number_of_mazes;

    int min_number_of_robots;
    cout << "What is minimum number of robots to simulate?\n";
    cin >> min_number_of_robots;

    int max_number_of_robots;
    cout << "What is maximum number of robots to simulate?\n";
    cin >> max_number_of_robots;

    string target_directory;
    cout << "Enter a directory to store results of the simulation:\n";
    cin >> target_directory;

    json simulation_info;
    // creating directories for simulation
    for(int i = min_number_of_robots; i <= max_number_of_robots; i++){
        std::filesystem::create_directories(target_directory + "sim_size_" + to_string(i)); // creating parent directories to store robot simulations of various swarm sizes
        simulation_info["Robot_Sizes"].push_back(i); // adding swarm size to simulation_info json 
    }

    vector<Coordinates> start_positions(max_number_of_robots, Coordinates(0,0));

    for(int j = 0; j < number_of_mazes; j++){ // for loops to run simulations

        for(int i = min_number_of_robots; i <= max_number_of_robots; i++){
            
            Maze m;
            m.generateRandomNxNMaze(maze_size, maze_size); // generating new random maze

            string directory_for_export = target_directory + "sim_size_" + to_string(i) + "/sim_" + to_string(j + 1) +  "/"; // getting directory for target output
            std::filesystem::create_directories(directory_for_export); // creating child directory to store robot simulation for this test

            runSimulation(&m, i, 3, &start_positions, directory_for_export); // running simulation
        }
    }

    exportJSON(simulation_info, "Sim_Settings", target_directory); // exporting info about simulations performed

    return;
}

int main(){
    // ~~~ Title printouts ~~~
    cout << "~~~ Multi-agent Robot Simulator ~~~\n";
    cout << "Created by Ryan Wiebe\n";
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
    cout << "Which of the following would you like to simulate?\n";
    cout << "1 - One time Simulation\n";
    cout << "2 - Test Cases\n";
    int input;

    cin >> input;

    switch(input){
        case 1:
        {
            simulateOneTime();
            break;
        }
        case 2:
        {
            testCases();
            break;
        }
        case 3:
        {
            testSwarmSize();
            break;
        }
        case 4:
        {
            testGroupSize();
            break;
        }
    }
    
    return 0;
}