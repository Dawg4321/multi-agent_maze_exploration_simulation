#include "MultiRobot_C_CellReservation.h"

MultiRobot_C_IE::MultiRobot_C_IE(int x, int y, RequestHandler* r, unsigned int xsize, unsigned int ysize): MultiRobot(x, y, r, xsize, ysize){

}

void MultiRobot_C_IE::robotLoop(GridGraph* maze){
    
    robotSetUp(); // call start up function before the robot loop

    while(robot_status != s_exit_loop){
        robotLoopStep(maze); // execute steps in loop until robot has reached exit state
    }

    return;
}

void MultiRobot_C_IE::robotSetUp(){

    assignIdFromMaster(); // getting id from robotmaster before begining robot exploration

    robot_status = s_stand_by; // initializing status to zero as robot is placed on standby until master returns ID within robot loop

    return;
}

int MultiRobot_C_IE::robotLoopStep(GridGraph* maze){
    
    robot_status = getMessagesFromMaster(robot_status); // checking if master wants robot to update status

    int status_of_execution = robot_status; // gathering robot_status before exectuion for return purposes
                                            // this must be done before execution as status may update after execution

    computeRobotStatus(maze); // compute a function based off of the robot's status     

    return status_of_execution;
}

int MultiRobot_C_IE::robotLoopStepforSimulation(GridGraph* maze){ // robot loop step used for simulation to allow for turn delays based off specific requests
                                                                  // this is meant to be used in conjunction with the turn system, used robotLoopStep if computing without turns

    robot_status = getMessagesFromMaster(robot_status); // checking if master wants robot to update status

    int status_of_execution = robot_status; // gathering robot_status before exectuion for return purposes
                                            // this must be done before execution as status may update after execution

    // place any states which require more than one turn here
    if(status_of_execution == s_compute_move || status_of_execution == s_scan_cell){ // if the current robot execution status computation must be delay
        return status_of_execution;                                                  // exit with status of execution and compute function from main()
    }

    computeRobotStatus(maze); // compute a function based off of the robot's status  

    return status_of_execution;
}

void MultiRobot_C_IE::computeRobotStatus(GridGraph* maze){ // function to compute the current robot's status

    switch(robot_status){
        case s_exit_loop: // exit status (fully shut off robot)
        {   
            // do nothing as loop will exit after this iteration
            
            break;
        }
        case s_shut_down: // shutdown mode
        {
            requestShutDown(); // notify master that robot is ready to shutdown

            robot_status = s_stand_by; // enter standy to await master response

            accepting_requests = false; // robot is no longer accepting requests as it has been told to shut down

            break;
        }
        case s_stand_by: // standby
        {   
            // do nothing
            
            break;
        }
        case s_scan_cell: // scan cell
        {   
            computeScanCell(maze); // computing scan cell

            break;
        }
        case s_pathfind: // planned path
        {   
            // repeat loop until cell which is being planned to has been reserved
            
            bool path_found = BFS_pf2NearestUnknownCell(&planned_path); // create planned path to nearest unknown cell
            
            if(path_found){
                requestReserveCell();
            
                robot_status = s_stand_by; // must wait for response
            }
            else{

                robot_status = s_pathfind; // must pathfind again as no unreserved cell found
            }
            
            break;
        }
        case s_move_robot: // request to move into a cell (checking if robot is occupying it)
        {
            requestMove2Cell(planned_path[0]);

            robot_status = s_stand_by; // must wait for response

            break;
        }
        case s_compute_move: // move robot 1 step
        {   
            computeMove(); // compute movement request

            break;
        }
        case s_pathfind2target:
        {
            Coordinates target_cell = getTarget2Pathfind();
            bool path_found = pf_BFS(target_cell.x,target_cell.y);

            if(path_found){  
                requestReserveCell(); // must reserve cell as path found
            }
            else{
                requestGetMap(); // must get map so robot can plan a path to the target cell
            }

            robot_status = s_stand_by; // must wait for response

            break;
        }
    }

    return;
}

int MultiRobot_C_IE::handleMasterResponse(Message* response, int current_status){

    current_status = handleCollisionResponse(response, current_status); // attempt to handle collision response
    current_status = handleCellReserveResponse(response, current_status); // attempt to handle reserve response

    int new_robot_status = MultiRobot::handleMasterResponse(response, current_status); // attempt to handle generic base responses

    return new_robot_status; // return changes to status
}

int MultiRobot_C_IE::handleMasterRequest(Message* response, int current_status){

    current_status = handleCollisionRequest(response, current_status); // attempt to handle collision request

    int new_robot_status = MultiRobot::handleMasterRequest(response, current_status); // attempt to handle generic base requests

    return new_robot_status; // return changes to status
}

