#include <iostream>
#include <pthread.h>

#include "Maze.h"
#include "RobotMaster.h"
#include "Robot.h"
#include "MultiRobot_NC_UI.h"
#include "MultiRobot_NC_IE.h"

using namespace std;

struct RobotArgs{ // structure to hold args for passing robot information into a new thread
    Robot* Generated_Robot; // dynamically allocated robot
    GridGraph Maze_Map; // Map of maze used by robot to scan cells

    RobotArgs(Robot* R1, GridGraph M){
        Generated_Robot = R1;
        Maze_Map.nodes = M.nodes;
        Maze_Map.x_edges = M.x_edges;
        Maze_Map.y_edges = M.y_edges;
    }
};

void* robotFunc(void* Robot_Info){ // function for robot running threads
    RobotArgs* Data = (RobotArgs*) Robot_Info;

    Robot* R = Data->Generated_Robot;

    R->robotLoop(&(Data->Maze_Map)); // run robot loop with specified maze info
}

void* controllerFunc(void* Robot_Controller){ // function to run Robot Controller in a seperate thread
    
    RobotMaster* RM = (RobotMaster*) Robot_Controller; // gathering passed in Robot_Controller

    RM->runRobotMaster(); // run RobotController until maze is mapped and robots have shut down fully
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
    }
}

RobotMaster* getNewRobotMaster(int robot_type, int number_of_robots, RequestHandler* request_handler, unsigned int xsize, unsigned int ysize){
    
    if(robot_type == 1 || robot_type == 2){ // if the robots to simulate are of type NC_UI or NC_IE
        return new RobotMaster(request_handler, number_of_robots, xsize, ysize);
    }
}

int main(){
    // ~~~ Title printouts ~~~
    cout << "~~~ Multi-agent Robot Simulator ~~~\n";
    cout << "Created by Ryan Wiebe\n";
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";

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
    
    int type_of_robots = 0;
    cin >> type_of_robots;

    
    // ~~~ Robot Master Thread Generation ~~~
    RequestHandler* request_handler = new RequestHandler(); // creating message handler for robot -> master communcation
    
    // gathering new RobotMaster compatible with specified type of robots
    RobotMaster* Robot_Controller = getNewRobotMaster(type_of_robots, number_of_robots, request_handler, Generated_Maze.getMazeXSize(), Generated_Maze.getMazeYSize());

     // creating thread to run Robot_Controller 
    pthread_t master_thread;
    pthread_create(&master_thread, NULL, &controllerFunc, (void*)Robot_Controller);

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
        Robot_Thread_Args[i] = new RobotArgs(Robots_Array[i], Generated_Maze.getMazeMap());
        // running robot thread
        pthread_create(&thread_id[i], NULL, &robotFunc, (void*)Robot_Thread_Args[i]);
    }

    // ~~~ Awaiting Robot Thread Completion  ~~~
    for(int i = 0; i < number_of_robots; i++) // waiting for robot threads to finish
        pthread_join(thread_id[i], NULL); 

    // ~~~ Awaiting Robot Master Thread Completion ~~~
    pthread_join(master_thread, NULL); // waiting for robot master thread to finish

    // ~~~ Deleting Dynamically Allocated Memory ~~

    delete Robot_Controller; // deleting RobotMaster
    
    for(int i = 0; i < number_of_robots; i++) // deleting all generated robots
        delete Robots_Array[i];

    delete request_handler; // deleting request handler used by robots

    return 0;
}