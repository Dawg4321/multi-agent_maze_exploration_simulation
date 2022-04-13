#include <iostream>
#include <fstream>
#include <pthread.h>

#include "Maze.h"
#include "RobotMaster_NC_UI.h"
#include "RobotMaster_NC_IE.h"
#include "RobotMaster_C_IE.h"
#include "MultiRobot_NC_UI.h"
#include "MultiRobot_NC_IE.h"
#include "MultiRobot_C_IE.h"

using namespace std;

using json = nlohmann::json; // simplifying namespace so "json" can be used instead of "nlohmann::json" when declaring json objects

struct TurnControlData{ // struct containing data used by robot and master threads to control turn flow
    
    pthread_barrier_t turn_start_barrier, turn_end_barrier; // barriers to synchronize robot threads into a turn format
    
    pthread_mutex_t finished_counter_mutex; // mutexes to protect the counters

    int finished_robots_counter; // tracks number of robots which have completed a turn

    const int number_of_robots; // constant value of number of robots being simulated

    TurnControlData(int num_robots): number_of_robots(num_robots){
        // initialzing barrier to number of robots + 1 (all robots + robot master must be waiting before next turn can start/finish)
        pthread_barrier_init(&turn_start_barrier, NULL, num_robots + 1);
        pthread_barrier_init(&turn_end_barrier, NULL, num_robots + 1);

        // intialzing pthread mutexes
        pthread_mutex_init(&finished_counter_mutex, NULL);

        finished_robots_counter = 0; // initializing finished_robots to 0 as no robots have finished executing fully
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

    RobotMasterArgs(RobotMaster* R1, TurnControlData* control_info){
        Generated_RobotMaster = R1;
        turn_control = control_info;
    }
};

struct RobotArgs{ // structure to hold args for passing robot information into a new thread
    Robot* Generated_Robot; // dynamically allocated robot
    GridGraph Maze_Map; // Map of maze used by robot to scan cells

    TurnControlData* turn_control; // struct containing info to control robot's turn

    RobotArgs(Robot* R1, GridGraph M, TurnControlData* control_info){
        Generated_Robot = R1;
        Maze_Map.nodes = M.nodes;
        Maze_Map.x_edges = M.x_edges;
        Maze_Map.y_edges = M.y_edges;
        turn_control = control_info;
    }
};

void* robotFunc(void* Robot_Info){ // function for robot running threads
    // gathering passed data
    RobotArgs* Data = (RobotArgs*) Robot_Info; // argument structure containing passed data
    Robot* R = Data->Generated_Robot; // robot to run in this thread
    TurnControlData* TurnControl = Data->turn_control; // turn control mechanisms

    R->robotSetUp(); // setting up robot before loop intialization

    int robot_execution_status = s_stand_by; // tracks what state the robot executed in last robot loop pass
    
    int number_of_turns_to_wait = 0; // tracks the number of turns robot has to sit out before executing a robotloop step
                                     // initialzing to zero as robot should not wait on its first turn
    
    while(robot_execution_status != s_exit_loop){ // continue executing turns until the robot has been put into shut down state

        pthread_barrier_wait(&TurnControl->turn_start_barrier); // waiting for all threads to complete preivous turn initialization before starting next turn

        if(number_of_turns_to_wait == 0){ // if robot does not have to sit out for a turn, execute robot loop step

            robot_execution_status = R->robotLoopStep(&Data->Maze_Map); // executing one step of the robot loop
 
            number_of_turns_to_wait++;
        }

        pthread_barrier_wait(&TurnControl->turn_end_barrier); // waiting for all threads to complete preivous turn initialization before starting next turn
        
        number_of_turns_to_wait--; // subtract number of turns to wait as robot has finished a turn
    }

    bool all_robots_done = false; // boolean to track whether all robots have completed their turns

    // safely incrementing finished_robots counter as robot is done exploring
    pthread_mutex_lock(&TurnControl->finished_counter_mutex);
    TurnControl->finished_robots_counter++;
    
    if(TurnControl->finished_robots_counter == TurnControl->number_of_robots){
        all_robots_done = true;
    }
    pthread_mutex_unlock(&TurnControl->finished_counter_mutex);

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
        string name = "Turn_";
        name += to_string(turn_counter);
        buffer_json[name] = RM->getRequestInfo(); // gathering json containing request info during this turn
        Data->turn_json.push_back(buffer_json);

        RM->clearRequestInfo(); // clearing contents of request info before next turn

        pthread_barrier_wait(&TurnControl->turn_start_barrier); // signalling robots to begin next turn
    }

    pthread_barrier_wait(&TurnControl->turn_end_barrier); // Signalling robot they can finally exit their loop after completion

    pthread_exit(NULL); // return from thread
}

Robot* getNewRobot(int robot_type, unsigned int x_pos, unsigned int y_pos, RequestHandler* request_handler, unsigned int xsize, unsigned int ysize){
    
    switch(robot_type){ // returning selected robot type
        case 1: // Selecting No Collision, Unintelligent Exploration
        {
            return new MultiRobot_NC_UI(x_pos, y_pos, request_handler, xsize, ysize);
        }
        case 2: // Selecting No Collision, Intelligent Exploration
        {
            return new MultiRobot_NC_IE(x_pos, y_pos, request_handler, xsize, ysize);
        }
        case 3:
        {
            return new MultiRobot_C_IE(x_pos, y_pos, request_handler, xsize, ysize);
        }
    }
}

RobotMaster* getNewRobotMaster(int robot_type, int number_of_robots, RequestHandler* request_handler, unsigned int xsize, unsigned int ysize){
    
    if(robot_type == 1){ // if the robots to simulate are of type NC_UI
        return new RobotMaster_NC_UI(request_handler, number_of_robots, xsize, ysize);
    }
    else if(robot_type == 2){ // if the robots to simulate are of type NC_IE
        return new RobotMaster_NC_IE(request_handler, number_of_robots, xsize, ysize);
    }
    else if(robot_type == 3){ // if the robots to simulate are of type C_IE
        return new RobotMaster_C_IE(request_handler, number_of_robots, xsize, ysize);
    }
}

bool exportJSON(json* json_2_export, string json_name){


    json_name += ".json"; // adding .json extension to passed in name

    std::ofstream json_file(json_name); // creating file stream to the json file

    if(json_file.is_open()){ // if file was created successfully

        json_file << std::setw(4) << *json_2_export << std::endl; // export json to .json file

        return true;
    }

    // failed to open file for export

    cout << "Error: Failed to write to " << json_name << ".json\n";

    return false;
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
            Generated_Maze.generateRandomNxNMaze(100, 100);
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
    RobotMasterArgs* RMArgs = new RobotMasterArgs(Robot_Controller, &turn_control_data);

    // creating thread to run Robot_Controller 
    pthread_t master_thread;
    pthread_create(&master_thread, NULL, &controllerFunc, (void*)RMArgs);

    // ~~~ Robot Thread Generation ~~~
    Robot* Robots_Array[number_of_robots]; // generating array for robots to be stored in
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
    
    exportJSON(&RMArgs->turn_json, "Simulation");

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
    RobotMaster* RM1 = new RobotMaster_NC_IE(req, num_robots, 9, 3);
    RobotMasterArgs RM1args(RM1, &turn_control_data);
    RM1->setGlobalMap(&g2);
    
    Robot* R1 = new MultiRobot_NC_IE(4, 0, req, 9, 3);
    RobotArgs R1args(R1, g1, &turn_control_data);
    R1->setLocalMap(&g2);
    Robot* R2 = new MultiRobot_NC_IE(5, 0, req, 9, 3);
    RobotArgs R2args(R2, g1, &turn_control_data);
    R2->setLocalMap(&g2);
    Robot* R3 = new MultiRobot_NC_IE(6, 0, req, 9, 3);
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
    exportJSON(&RM1args.turn_json, "Test_Case");

    // deleting dynamically allocated data
    delete RM1;
    delete R1;
    delete R2;
    delete R3;
    delete req;

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
    }
    
    return 0;
}