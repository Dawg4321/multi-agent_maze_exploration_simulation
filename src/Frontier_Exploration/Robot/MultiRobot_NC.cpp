#include "MultiRobot_NC.h"

MultiRobot_NC::MultiRobot_NC(unsigned int x, unsigned int y, RequestHandler* r, unsigned int xsize, unsigned int ysize): MultiRobot(x, y, r, xsize, ysize){

}

void MultiRobot_NC::robotLoop(GridGraph* maze){

    robotSetUp(); // call start up function before the robot loop

    while(robot_status != s_exit_loop){
        robotLoopStep(maze); // execute steps in loop until robot has reached exit state
    }

    return;
}

void MultiRobot_NC::robotSetUp(){

    assignIdFromMaster(); // getting id from robotmaster before begining robot exploration

    robot_status = s_stand_by; // initializing status to zero as robot is placed on standby until master returns ID within robot loop

    return;
}

int MultiRobot_NC::robotLoopStep(GridGraph* maze){

    robot_status = getMessagesFromMaster(robot_status); // checking if master wants robot to update status

    int status_of_execution = robot_status; // gathering robot_status before exectuion for return purposes
                                            // this must be done before execution as status may update after execution

    computeRobotStatus(maze);     

    return status_of_execution; // return what robot did during this iteration of the loop
}

int MultiRobot_NC::robotLoopStepforSimulation(GridGraph* maze){ // robot loop step used for simulation to allow for turn delays based off specific requests
                                                                   // this is meant to be used in conjunction with the turn system, used robotLoopStep if computing without turns

    robot_status = getMessagesFromMaster(robot_status); // checking if master wants robot to update status

    int status_of_execution = robot_status; // gathering robot_status before exectuion for return purposes
                                            // this must be done before execution as status may update after execution

    // place any states which require more than one turn here
    if(status_of_execution == s_scan_cell){ // if the current robot execution status computation must be delay
        return status_of_execution;         // exit with status of execution and compute function from main()
    }
    else if(status_of_execution == s_move_robot){ // need to handle special case if robot is computing a move
        return s_compute_move;                    // this needs to be done as simulator has been programmed to use s_compute_move as the status to trigger movement delay
    }

    computeRobotStatus(maze); // compute a function based off of the robot's status  

    return status_of_execution;
}

void MultiRobot_NC::computeRobotStatus(GridGraph* maze){
    
    switch(robot_status){
        case s_exit_loop: // exit status (fully shut off robot)
        {
            // do nothing as robot will exit in this state
            
            break;
        }
        case s_shut_down: // shutdown mode
        {
            requestShutDown(); // notify master that robot is ready to shutdown
            
            robot_status = s_stand_by; // enter standy to await master response 

            break;
        }
        case s_stand_by: // standby
        {   
            // do nothing
            
            break;
        }
        case s_scan_cell: // scan cell and update master with cell info
        {   
            std::vector<bool> connection_data = scanCell(maze); // scan cell which is occupied by the robot 

            requestGlobalMapUpdate(connection_data); // sending message to master with scanned maze information

            robot_status = s_pathfind; // setting status to 2 so pathfinding will occur on next loop cycle

            break;
        }
        case s_pathfind: // generating path to nearest unknown cell
        {   
                
            BFS_pf2NearestUnknownCell(&planned_path); // create planned path to nearest unknown cell
            
            robot_status = s_move_robot; // setting status to 3 so movement will occur on next loop cycle

            break;
        }
        case s_move_robot: // move robot to next cell in planned bath
        {   

            bool move_occured = MultiRobot::move2Cell(planned_path[0]); // attempt move robot to next location in planned path queue

            if(move_occured){ // if movement succeed 
                planned_path.pop_front(); // remove element at start of planned path queue as it has occured 
            
                if(planned_path.empty()){ // if there are no more moves to occur, must be at an unscanned cell
                    robot_status = s_scan_cell; // set robot to scan cell on next loop iteration as at desination cell
                }
                else{ // if more moves left, keep status to 3
                    robot_status = s_move_robot; 
                }
            }
            else{ // if movement failed
                    planned_path.clear(); // clearing current planned path
                    robot_status = s_pathfind; // attempt to plan a new path which will hopefully not cause movement to fail
            }

            break;
        }
    }

    return;
}