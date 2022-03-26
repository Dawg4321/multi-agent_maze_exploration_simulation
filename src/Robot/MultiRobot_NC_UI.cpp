#include "MultiRobot_NC_UI.h"

MultiRobot_NC_UI::MultiRobot_NC_UI(unsigned int x, unsigned int y, RequestHandler* r, unsigned int xsize, unsigned int ysize): MultiRobot(x, y, r, xsize, ysize){

}

void MultiRobot_NC_UI::robotLoop(GridGraph* maze){
    
    assignIdFromMaster(); // getting id from robotmaster before begining robot exploration
    
    int status = 0; // tracks status of robot
                    // -1 = shut down
                    // 0 = stand by
                    // 1 = explore
                    // initializing to zero as robot is placed on standby until master updates status 

    bool loop_break = false; // bool to break for loop if shutdown has occured

    while(!loop_break){
        
        status = getRequestsFromMaster(status); // checking if master wants robot to update status

        switch(status){
            case -1: // shutdown mode
            {
                requestShutDown(); // notify master that robot is ready to shutdown
                loop_break = true; // setting loop break to ensure break while loop
                break;
            }
            case 0: // standby
            {   
                // do nothing
                break;
            }
            case 1: // scan cell and update master with cell info
            {   
                std::vector<bool> connection_data = scanCell(maze); // scan cell which is occupied by the robot 

                requestGlobalMapUpdate(connection_data); // sending message to master with scanned maze information

                status = 2; // setting status to 2 so pathfinding will occur on next loop cycle

                break;
            }
            case 2: // generating path to nearest unknown cell
            {   
                  
                BFS_pf2NearestUnknownCell(&planned_path); // create planned path to nearest unknown cell
 
                status = 3; // setting status to 3 so movement will occur on next loop cycle

                break;
            }
            case 3: // move robot to next cell in planned bath
            {   
                bool move_occured = MultiRobot::move2Cell(planned_path[0]); // move robot to next location in planned path queue

                if(move_occured){ // if movement succeed 
                    planned_path.pop_front(); // remove element at start of planned path queue as it has occured 
                
                    if(planned_path.empty()){ // if there are no more moves to occur, must be at an unscanned cell
                        status = 1; // set robot to scan cell on next loop iteration as at desination cell
                    }
                    else{ // if more moves left, keep status to 3
                        status = 3;
                    }
                }
                else{ // if movement failed
                      planned_path.clear(); // clearing current planned path
                      status = 2; // attempt to plan a new path which will hopefully not cause movement to faile
                }

                break;
            }
        }        
    }

    return;
}