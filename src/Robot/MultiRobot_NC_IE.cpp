#include "MultiRobot_NC_IE.h"

MultiRobot_NC_IE::MultiRobot_NC_IE(unsigned int x, unsigned int y, RequestHandler* r, unsigned int xsize, unsigned int ysize): MultiRobot(x, y, r, xsize, ysize){

}

void MultiRobot_NC_IE::robotLoop(GridGraph* maze){
    
    assignIdFromMaster(); // getting id from robotmaster before begining robot exploration
    
    int status = 0; // tracks status of robot
                    // -1 = shut down
                    // 0 = stand by
                    // 1 = explore
                    // initializing to zero as robot is placed on standby until master updates status 

    bool done_exploring = false; // bool to break for loop if shutdown has occured

    while(!done_exploring){
        
        status = getMessagesFromMaster(status); // checking if master wants robot to update status

        switch(status){
            case s_exit_loop: // exit status (fully shut off robot)
            {
                done_exploring = true; // exit loop
                
                break;
            }
            case s_shut_down: // shutdown mode
            {
                requestShutDown(); // notify master that robot is ready to shutdown

                status = s_stand_by; // enter standy to await master response 

                break;
            }
            case s_stand_by: // standby
            {   
                // do nothing
                status = s_stand_by; // stay in standby until updated by incoming request
                break;
            }
            case s_scan_cell: // scan cell
            {   
                std::vector<bool> connection_data = scanCell(maze); // scan cell which is occupied by the robot 

                requestGlobalMapUpdate(connection_data); // sending message to master with scanned maze information

                status = s_pathfind; // setting status to 2 so pathfinding will occur on next loop cycle

                break;
            }
            case s_pathfind: // planned path
            {   
                bool cell_reserved = false;

                // repeat loop until cell which is being planned to has been reserved
                
                BFS_pf2NearestUnknownCell(&planned_path); // create planned path to nearest unknown cell
                    
                MultiRobot::requestReserveCell();

                status = s_stand_by; // must wait for response
                
                break;
            }
            case s_move_robot: // move robot 1 step
            {   
                bool move_occured = MultiRobot::move2Cell(planned_path[0]); // attempt to move robot to next location in planned path queue

                if(move_occured){ // if movement succeed 
                    planned_path.pop_front(); // remove element at start of planned path queue as it has occured 
                
                    if(planned_path.empty()){ // if there are no more moves to occur, must be at an unscanned cell
                        status = s_scan_cell; // set robot to scan cell on next loop iteration as at desination cell
                    }
                    else{ // if more moves left, keep status to 3
                        status = s_move_robot;
                    }
                }
                else{ // if movement failed
                      planned_path.clear(); // clearing current planned path
                      status = s_pathfind; // attempt to plan a new path which will hopefully not cause movement to faile
                }
                break;
            }
        }        
    }

    return;
}