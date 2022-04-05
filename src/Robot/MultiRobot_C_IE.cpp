#include "MultiRobot_C_IE.h"

MultiRobot_C_IE::MultiRobot_C_IE(unsigned int x, unsigned int y, RequestHandler* r, unsigned int xsize, unsigned int ysize): MultiRobot(x, y, r, xsize, ysize){

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

            break;
        }
        case s_stand_by: // standby
        {   
            // do nothing
            
            break;
        }
        case s_scan_cell: // scan cell
        {   
            std::vector<bool> connection_data = scanCell(maze); // scan cell which is occupied by the robot 

            requestGlobalMapUpdate(connection_data); // sending message to master with scanned maze information

            robot_status = s_pathfind; // setting status to 2 so pathfinding will occur on next loop cycle

            break;
        }
        case s_pathfind: // planned path
        {   
            bool cell_reserved = false;

            // repeat loop until cell which is being planned to has been reserved
            
            BFS_pf2NearestUnknownCell(&planned_path); // create planned path to nearest unknown cell
                
            requestReserveCell();

            robot_status = s_stand_by; // must wait for response
            
            break;
        }
        case s_request_move: // request to move into a cell (checking if robot is occupying it)
        {
            requestMove2Cell(planned_path[0]);

            robot_status = s_stand_by; // must wait for response

            break;
        }
        case s_move_robot: // move robot 1 step
        {   
            bool move_occured = MultiRobot::move2Cell(planned_path[0]); // attempt to move robot to next location in planned path queue

            if(move_occured){ // if movement succeed 
                planned_path.pop_front(); // remove element at start of planned path queue as it has occured 
            
                if(planned_path.empty()){ // if there are no more moves to occur, must be at an unscanned cell
                    robot_status = s_scan_cell; // set robot to scan cell on next loop iteration as at desination cell
                }
                else{ // if more moves left, try another movement
                    robot_status = s_request_move;
                }
            }
            else{ // if movement failed
                    planned_path.clear(); // clearing current planned path
                    robot_status = s_pathfind; // attempt to plan a new path which will hopefully not cause movement to faile
            }
            break;
        }
    }

    return status_of_execution;
}

int MultiRobot_C_IE::handleMasterResponse(Message* response, int current_status){

    current_status = handleCollisionResponse(response, current_status); // attempt to handle collision response
    current_status = handleCellReserveResponse(response, current_status); // attempt to handle reserve response

    int new_robot_status = MultiRobot::handleMasterResponse(response, current_status); // attempt to handle generic base responses

    return new_robot_status; // return changes to status
}